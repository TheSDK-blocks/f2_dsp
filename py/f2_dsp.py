# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 14.11.2017 19:50
import numpy as np
import scipy.signal as sig
import tempfile
import subprocess
import shlex
import time

from refptr import *
from thesdk import *
from rtl import *

#Simple buffer template
class f2_dsp(rtl,thesdk):
    def __init__(self,*arg): 
        self.proplist = [ 'Rs', 'Rs_dsp', 'Hstf', 'Hltf' ];    #properties that can be propagated from parent
        self.Rs = 160e6;                 # sampling frequency
        self.Rs_dsp=20e6
        self.Hstf=1                      #filters for sybol sync
        self.Hltf=1
        self.iptr_A = refptr();
        self.model='py';             #can be set externally, but is not propagated
        self._Z = refptr();
        self._Frame_sync_short = refptr();
        self._Frame_sync_long = refptr();
        self._classfile=__file__
        if len(arg)>=1:
            parent=arg[0]
            self.copy_propval(parent,self.proplist)
            self.parent =parent;
    def init(self):
        self.def_rtl()
        rndpart=os.path.basename(tempfile.mkstemp()[1])
        self._infile=self._rtlsimpath +'/A_' + rndpart +'.txt'
        self._outfile=self._rtlsimpath +'/Z_' + rndpart +'.txt'
        self._rtlcmd=self.get_rtlcmd()

    def get_rtlcmd(self):
        #the could be gathered to rtl class in some way but they are now here for clarity
        submission = ' bsub -q normal '  
        rtllibcmd =  'vlib ' +  self._workpath + ' && sleep 2'
        rtllibmapcmd = 'vmap work ' + self._workpath

        if (self.model is 'vhdl'):
            rtlcompcmd = ( 'vcom ' + self._rtlsrcpath + '/' + self._name + '.vhd '
                          + self._rtlsrcpath + '/tb_'+ self._name+ '.vhd' )
            rtlsimcmd =  ( 'vsim -64 -batch -t 1ps -g g_infile=' + 
                           self._infile + ' -g g_outfile=' + self._outfile 
                           + ' work.tb_' + self._name + ' -do "run -all; quit -f;"')
            rtlcmd =  submission + rtllibcmd  +  ' && ' + rtllibmapcmd + ' && ' + rtlcompcmd +  ' && ' + rtlsimcmd

        elif (self.model is 'sv'):
            rtlcompcmd = ( 'vlog -work work ' + self._rtlsrcpath + '/' + self._name + '.sv '
                           + self._rtlsrcpath + '/tb_' + self._name +'.sv')
            rtlsimcmd = ( 'vsim -64 -batch -t 1ps -voptargs=+acc -g g_infile=' + self._infile
                          + ' -g g_outfile=' + self._outfile + ' work.tb_' + self._name  + ' -do "run -all; quit;"')

            rtlcmd =  submission + rtllibcmd  +  ' && ' + rtllibmapcmd + ' && ' + rtlcompcmd +  ' && ' + rtlsimcmd

        else:
            rtlcmd=[]
        return rtlcmd

    def run(self,*arg):
        if len(arg)>0:
            par=True      #flag for parallel processing
            queue=arg[0]  #multiprocessing.Queue as the first argument
        else:
            par=False

        if self.model=='py':
            resampled=np.array(self.iptr_A.Value[0::int(self.Rs/self.Rs_dsp)],ndmin=2)
            #out=resampled.T
            
            #Matched filtering for short and long sequences and squaring for energy
            #Scale according to l2 norm

            #The maximum of this is sum of squares squared
            matchedshort=np.abs(sig.convolve(resampled.T, self.Hstf, mode='full'))**2
            matchedlong=(np.sum(np.abs(self.Hstf)**2)/np.sum(np.abs(self.Hltf)**2)*np.abs(sig.convolve(resampled.T, self.Hltf, mode='full')))**2

            #Dummy filter to delay the payload signal
            dfil1=np.zeros(self.Hltf.shape)
            dfil1[-1,0]=1
            delayed=sig.convolve(resampled.T, dfil1, mode='full')
            
            #Filter for energy filtering (average of 6 samples)
            Efil=np.ones((6,1))
            Sshort=sig.convolve(matchedshort,Efil,mode='full')
            Slong=sig.convolve(matchedlong,Efil,mode='full')

            #Dummy filter to delay the payload signal
            dfil2=np.zeros((6,1))
            dfil2[-1,0]=1
            delayed=sig.convolve(delayed, dfil2, mode='full')
            out=delayed

            #Sum of the 4 past spikes with separation of 16 samples
            Sfil=np.zeros((65,1))
            Sfil[16:65:16]=1
            Sspikes_short=sig.convolve(Sshort,Sfil,mode='full')/np.sum(np.abs(Sfil))

            #Compensate the matched long for the filter delays of the short
            delay=len(Sspikes_short)-len(Slong)
            Slong=np.r_['0',Slong, np.zeros((delay,1))]
            Sspikes_sum=Slong+Sspikes_short

            #TODO: Check the sequence formation. Still some weirdness in the spike magnitudes 
            #TODO: Check the effect of interpolation filtering 
            #Find the frame start
            found=False
            i=0
            Smaxprev=0
            indmaxprev=0
            while not found and i+16<= len(Sspikes_sum):
                Testwin=(Sspikes_sum[i:i+16])
                Smax=np.max(Testwin)
                indmax=i+np.argmax(Testwin,axis=0)
                
                if Smax != Sspikes_sum[indmax]:
                    self.print_log({'type':'F', 'msg': "Something wrong with the symbol boundary"})

                if Smax < 0.85 * Smaxprev:
                    found = True
                    Smax=Smaxprev
                    indmax=indmaxprev
                    self.print_log({'type':'I', 'msg': "Found the Symbol boundary at sample %i" %(indmax)})
                    
                else:
                    Smaxprev=Smax
                    indmaxprev=indmax
                    i+=16

            #Need to compensate for the delays caused by the filters
            startind=int(indmax)
            self.print_log({'type':'I', 'msg': "Start of the long preamble sequence is at %i" %(startind)})


            #Strip the cyclic prefix and take two sequences 
            long_sequence=delayed[startind+16:startind+16+128,0].T
            long_seq_freq=long_sequence.reshape((-1,64))
            long_seq_freq=np.fft.fft(long_seq_freq,axis=1)
            long_seq_freq=np.r_['1', long_seq_freq[:,-32:-1], long_seq_freq[:,0:32]]
            

            #Start the OFDM demodulation here
            payload=delayed[startind+160::]
            length=int(np.floor(payload.shape[0]/80)*80)
            payload=payload[0:length,:]
            payload=payload.reshape((-1,80))
            print(payload.shape)
            payload=payload[:,16::]
            demod=np.fft.fft(payload,axis=1)
            demod=np.r_['1', demod[:,-32:-1], demod[:,0:32]]
            demod=demod.reshape(-1,1)

            if par:
                queue.put(demod)
                queue.put(Sspikes_short)
                queue.put(Sspikes_sum)
                #queue.put(Slong)

            self._Z.Value=demod
            self._Frame_sync_short.Value=Sspikes_short
            self._Frame_sync_long.Value=Sspikes_sum
            #self._Frame_sync_long.Value=Slong
            #Next we need to calculate the delays for the payload signal
        else: 
          try:
              os.remove(self._infile)
          except:
              pass
          fid=open(self._infile,'wb')
          np.savetxt(fid,np.transpose(self.iptr_A.Value),fmt='%.0f')
          #np.savetxt(fid,np.transpose(inp),fmt='%.0f')
          fid.close()
          while not os.path.isfile(self._infile):
              #print("Wait infile to appear")
              time.sleep(1)
          try:
              os.remove(self._outfile)
          except:
              pass
          print("Running external command \n", self._rtlcmd , "\n" )
          subprocess.call(shlex.split(self._rtlcmd));
          
          while not os.path.isfile(self._outfile):
              #print("Wait outfile to appear")
              time.sleep(1)
          fid=open(self._outfile,'r')
          #fid=open(self._infile,'r')
          #out = .np.loadtxt(fid)
          out = np.transpose(np.loadtxt(fid))
          fid.close()
          if par:
              queue.put(out)
          self._Z.Value=out
          os.remove(self._infile)
          os.remove(self._outfile)

