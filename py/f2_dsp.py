# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 17.11.2017 14:26
import numpy as np
import scipy.signal as sig
import tempfile
import subprocess
import shlex
import time

from refptr import *
from thesdk import *
from rtl import *
import signal_generator_802_11n as sg80211n


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
        self.DEBUG= False
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
            self.print_log({'type':'D', 'msg':"sskiggebyy"})
            self.print_log({'type':'D', 'msg':self.iptr_A.Value[320+16:320+80]}) 
            Freqmap=range(-32,32)
            test=self.iptr_A.Value[320+16:320+80]
            test.shape=(-1,1)
            self.print_log({'type':'D', 'msg':test.shape})
            test=np.fft.fft(test,axis=0)/64
            self.print_log({'type':'D', 'msg':test[Freqmap]})
            #self.print_log({'type':'D', 'msg':resampled[0,160:180,0]}) 
            #Matched filtering for short and long sequences and squaring for energy
            #Scale according to l2 norm

            #The maximum of this is sum of squares squared
            matchedshort=np.abs(sig.convolve(resampled.T, self.Hstf, mode='full'))**2
            matchedlong=(np.sum(np.abs(self.Hstf)**2)/np.sum(np.abs(self.Hltf)**2)*np.abs(sig.convolve(resampled.T, self.Hltf, mode='full')))**2

            #Dummy filter to delay the payload signal
            dfil1=np.zeros(self.Hltf.shape)
            dfil1[-1,0]=1
            delayed=sig.convolve(resampled.T, dfil1, mode='full')
            self.print_log({'type':'D', 'msg':"testviis"})
            self.print_log({'type':'D', 'msg':delayed[320+16+len(dfil1)-1:320+len(dfil1)-1+80]})
            test=(delayed[320+16+len(dfil1)-1:320+len(dfil1)-1+80])
            test.shape=(-1,1)
            self.print_log({'type':'D', 'msg':test.shape})
            test=np.fft.fft(test,axis=0)/64
            self.print_log({'type':'D', 'msg':test[Freqmap]})
            #Filter for energy filtering (average of 6 samples)
            Efil=np.ones((6,1))
            Sshort=sig.convolve(matchedshort,Efil,mode='full')
            Slong=sig.convolve(matchedlong,Efil,mode='full')

            #Dummy filter to delay the payload signal
            dfil2=np.zeros((6,1))
            dfil2[-1,0]=1
            delayed=sig.convolve(delayed, dfil2, mode='full')
            out=delayed
            self.print_log({'type':'D', 'msg':"testkuus"})
            self.print_log({'type':'D', 'msg':delayed[320+16+len(dfil1)-1+len(dfil2)-1:320+len(dfil1)-1+len(dfil2)-1+80]})
            test=(delayed[320+16+len(dfil1)-1+len(dfil2)-1:320+len(dfil1)-1+len(dfil2)-1+80])
            test.shape=(-1,1)
            self.print_log({'type':'D', 'msg':test.shape})
            test=np.fft.fft(test,axis=0)/64
            self.print_log({'type':'D', 'msg':test[Freqmap]})
            #self.print_log({'type':'D', 'msg':len(dfil1)+len(dfil2)})
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

            #Ofdm manipulations start here
            ofdm64dict=sg80211n.ofdm64dict_noguardband

            #Strip the cyclic prefix and take two sequences 
            self.print_log({'type':'D', 'msg':"Testseiska"})
            self.print_log({'type':'D', 'msg':startind+3+160+16})
            self.print_log({'type':'D', 'msg':320+16+len(dfil1)-1+len(dfil2)-1})
            self.print_log({'type':'D', 'msg':"fft"})

            test=delayed[startind+3+160+16:startind+3+160+80]
            #test=(delayed[320+16+len(dfil1)-1+len(dfil2)-1:320+len(dfil1)-1+len(dfil2)-1+80])
            test.shape=(-1,1)
            test=np.fft.fft(test,axis=0)/64
            self.print_log({'type':'D', 'msg':test.shape})
            Freqmap=range(-32,32)
            self.print_log({'type':'D', 'msg':test[Freqmap]})
            
            #long_sequence=delayed[startind+3+16:startind+3+16+128,0].T
            long_sequence=delayed[startind+3+32:startind+3+32+128,0].T
            long_seq_freq=long_sequence.reshape((-1,64))
            long_seq_freq=np.sum(long_seq_freq,axis=0)
            self.print_log({'type':'D', 'msg':long_seq_freq})
            #long_seq_freq=np.fft.fft(long_seq_freq,axis=1)
            long_seq_freq=np.fft.fft(long_seq_freq)
            #long_seq_freq=np.fft.fft(long_seq_freq[:,sg80211n.Freqmap],axis=1)

            #Map the negative frequencies to the beginning of the array
            #long_seq_freq=long_seq_freq[:,sg80211n.Freqmap]
            long_seq_freq=long_seq_freq[sg80211n.Freqmap]
            #self.print_log({'type':'D', 'msg':long_seq_freq}) 
            #Estimate the channel

            #channel_est=np.multiply(np.sum(long_seq_freq,axis=0),sg80211n.PLPCsyn_long.T)
            channel_est=np.multiply(long_seq_freq,sg80211n.PLPCsyn_long.T)
            self.print_log({'type':'D', 'msg':"Channel estimate is %s " %(20*np.log10(np.abs(channel_est)/np.max(np.abs(channel_est))))}  )
            channel_corr=np.zeros_like(channel_est)
            
            data_loc=sg80211n.ofdm64dict_withguardband['data_loc']
            pilot_loc=sg80211n.ofdm64dict_withguardband['pilot_loc']
            data_and_pilot_loc=np.sort(np.r_[data_loc, pilot_loc])

            channel_corr[0,data_and_pilot_loc+32]=1/channel_est[0,data_and_pilot_loc+32]
            self.print_log({'type':'D', 'msg':"Corrected channel should be is %s " %(20*np.log10(np.abs(channel_corr*channel_est)))}  )
            #Start the OFDM demodulation here
            # Additional 3 is simulated to provide ideal reception
            payload=delayed[startind+3+160::]
            length=int(np.floor(payload.shape[0]/80)*80)
            self.print_log({'type':'D', 'msg':payload.shape})
            payload=payload[0:length,0]
            payload.shape=(1,-1)
            payload=payload.reshape((-1,80))
            
            #self.print_log({'type':'D', 'msg':payload.shape})
            payload=payload[:,16::]
            self.print_log({'type':'D', 'msg':"testkasi"})
            self.print_log({'type':'D', 'msg':payload.shape})
            test=payload[0,:]
            test.shape=(-1,1)
            test=np.fft.fft(test,axis=0)/64
            self.print_log({'type':'D', 'msg':test.shape})
            Freqmap=range(-32,32)
            self.print_log({'type':'D', 'msg':test[Freqmap]})
            self.print_log({'type':'D', 'msg':"payload"})
            self.print_log({'type':'D', 'msg':payload[0,:]})
            self.print_log({'type':'D', 'msg':payload.shape})
            demod=np.fft.fft(payload,axis=1)
            #demod=demod[:,sg80211n.Freqmap]
            demod=demod[:,Freqmap]
            self.print_log({'type':'D', 'msg':demod[0,:]/64})
            self.print_log({'type':'D', 'msg':np.multiply(demod[0,:],channel_corr)})
            corr_mat=np.ones((demod.shape[0],1))@channel_corr
            self.print_log({'type':'D', 'msg':corr_mat[0,:]})
            demod=np.multiply(demod,corr_mat)
            self.print_log({'type':'D', 'msg':demod[0,:]})
            demod=demod[:,data_and_pilot_loc+32]
            test=np.mean(np.abs(demod),axis=0)
            self.print_log({'type':'D', 'msg':test})
            demod=demod.reshape(-1,1)
            #(LI, LQ)=calculate_evm({'signal':demod, 'QAM':16 })
            #self.print_log({'type':'D', 'msg':(LI,LQ)})
            
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
              self.print_log({'type':'I', 'msg':"Wait infile to appear"})
              time.sleep(1)
          try:
              os.remove(self._outfile)
          except:
              pass
          self.print_log({'type':'I', 'msg':"Running external command %s\n" %(self._rtlcmd) })
          subprocess.call(shlex.split(self._rtlcmd));
          
          while not os.path.isfile(self._outfile):
              self.print_log({'type':'I', 'msg':"Wait outfile to appear"})
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

#Define functions
def calculate_evm(argdict):
    signal=argdict['signal']
    signal.shape=(-1,1)
    QAM=argdict['QAM']
    levels=int(np.sqrt(QAM))
    I=np.real(signal)
    Q=np.imag(signal)

    #Constellation should be symmetric along the baseline 
    LI=[]
    LQ=[]
    for i in range(int(levels/2)):
        meanI=np.mean(I,axis=0)
        meanQ=np.mean(Q,axis=0)
        LI=np.r_[LI, meanI]
        LQ=np.r_[LQ, meanQ]
        I=np.abs(I-meanI)
        Q=np.abs(Q-meanQ)
    return (LI, LQ)
