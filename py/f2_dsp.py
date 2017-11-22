# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 21.11.2017 11:59
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
        self.model='py';                 #can be set externally, but is not propagated
        self.par= False                  #By default, no parallel processing
        self.queue= []                   #By default, no parallel processing
        self._decimated=[]               #signals sampled at Rs_dsp
        self._delayed=[]
        self._sync_index=[]              #Signal index for the symbol synchronization
        self._channel_est=[]
        self._channel_corr=[]
        self._symbols = refptr();
        self._wordstream = refptr();
        self._bitstream = refptr();
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
            self.par=True      #flag for parallel processing
            self.queue=arg[0]  #multiprocessing.Queue as the first argument
        else:
            self.par=False

        if self.model=='py':
            self.decimate_input()
            self.delay_input()
            self.symbol_sync()
            self.estimate_channel()
            self.receive_symbols()
            self.extract_data()

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
          if self.par:
              self.queue.put(out)
          self._Z.Value=out
          os.remove(self._infile)
          os.remove(self._outfile)

    def decimate_input(self):
        #This is teh simplest form of decimation
        self._decimated=np.array(self.iptr_A.Value[0::int(self.Rs/self.Rs_dsp)],ndmin=2)
        self._decimated.shape=((-1,1)) 
    
    def delay_input(self):
        #Dummy filter to delay the payload signal in paralled with symbol sync
        dfil1=np.zeros(self.Hltf.shape)
        dfil1[-1,0]=1
        self.delayed=sig.convolve(self._decimated, dfil1, mode='full')

        #Dummy filter to delay the payload signal
        dfil2=np.zeros((6,1))
        dfil2[-1,0]=1
        self._delayed=sig.convolve(self.delayed, dfil2, mode='full')

    def symbol_sync(self):
        #Matched filtering for short and long sequences and squaring for energy
        #Scale according to l2 norm

        #The maximum of this is sum of squares squared
        matchedshort=np.abs(sig.convolve(self._decimated, self.Hstf, mode='full'))**2
        matchedlong=(np.sum(np.abs(self.Hstf)**2)/np.sum(np.abs(self.Hltf)**2)*np.abs(sig.convolve(self._decimated, self.Hltf, mode='full')))**2

        #Filter for energy filtering (average of 6 samples)
        Efil=np.ones((6,1))
        Sshort=sig.convolve(matchedshort,Efil,mode='full')
        Slong=sig.convolve(matchedlong,Efil,mode='full')

        #Sum of the 4 past spikes with separation of 16 samples
        Sfil=np.zeros((65,1))
        Sfil[16:65:16]=1
        Sspikes_short=sig.convolve(Sshort,Sfil,mode='full')/np.sum(np.abs(Sfil))

        #Compensate the matched long for the filter delays of the short
        delay=len(Sspikes_short)-len(Slong)
        Slong=np.r_['0',Slong, np.zeros((delay,1))]
        Sspikes_sum=Slong+Sspikes_short
        self.Sspikes_short=Sspikes_short

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
        if self.par:
            self.queue.put(Sspikes_short)
            self.queue.put(Sspikes_sum)
        self._sync_index=int(indmax)+int(Efil.shape[0]/2)
        self._Frame_sync_short.Value=Sspikes_short
        self._Frame_sync_long.Value=Sspikes_sum
        self.print_log({'type':'I', 'msg': "Start of the long preamble sequence is at %i" %(self._sync_index)})

    def estimate_channel(self):
        #Offset is used to position the sampling istant in the middle of cyclic prefixes
        #of the payload symbols
        #If delay is not correct, it is visible as phase offset and is compensated by
        #channel egualization, as long as the delay offset is the same for long sequence and
        #payload frames
        offset=int(sg80211n.TGI2/2)+int(sg80211n.ofdm64dict_noguardband['CPlen']/2)
        long_sequence=self._delayed[self._sync_index+offset:self._sync_index+offset+128,0].T
        long_seq_freq=long_sequence.reshape((-1,64))
        long_seq_freq=np.sum(long_seq_freq,axis=0)
        long_seq_freq=np.fft.fft(long_seq_freq)

        #Map the negative frequencies to the beginning of the array
        long_seq_freq=long_seq_freq[sg80211n.Freqmap]

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
        self._channel_est=channel_est
        self._channel_corr=channel_corr

    def receive_symbols(self):
        #Ofdm manipulations start here
        #Start the OFDM demodulation here
        offset=int(sg80211n.TGI2/2)+int(sg80211n.ofdm64dict_noguardband['CPlen']/2)
        data_loc=sg80211n.ofdm64dict_withguardband['data_loc']
        pilot_loc=sg80211n.ofdm64dict_withguardband['pilot_loc']
        data_and_pilot_loc=np.sort(np.r_[data_loc, pilot_loc])
        cyclicsymlen=int(sg80211n.ofdm64dict_noguardband['framelen']+sg80211n.ofdm64dict_noguardband['CPlen'])
        payload=self._delayed[self._sync_index+offset+2*len(sg80211n.PLPCsyn_long)::]
        length=int(np.floor(payload.shape[0]/cyclicsymlen)*(cyclicsymlen))
        payload=payload[0:length,0]
        payload.shape=(1,-1)
        payload=payload.reshape((-1,cyclicsymlen))
        
        #Strip the cyclic prefix
        payload=payload[:,sg80211n.ofdm64dict_noguardband['CPlen']::]
        demod=np.fft.fft(payload,axis=1)
        demod=demod[:,sg80211n.Freqmap]
        corr_mat=np.ones((demod.shape[0],1))@self._channel_corr
        demod=np.multiply(demod,corr_mat)
        demod=demod[:,data_and_pilot_loc+int(sg80211n.ofdm64dict_noguardband['framelen']/2)]
        demod=demod.reshape(-1,1)
        #self.symbols=demod

        if self.par:
            self.queue.put(demod)
        self._symbols.Value=demod

    def extract_data(self):
        QAM=16
        maxval=int(np.sqrt(QAM))-1
        #Scaling for quantization
        conststd=np.sqrt(np.sum(np.arange(1,maxval+1,2)**2))/len(np.arange(1,maxval+1,2))
        
        #Shift to positive and quantize
        normalized=self._symbols.Value/np.std(self._symbols.Value)*conststd+maxval/2*(1+1j)
        #Quantize modulation levels in multiples of 1
        realwordstream = np.clip(np.round(np.real(normalized)), 0, maxval).astype(int)
        imagwordstream = np.clip(np.round(np.imag(normalized)), 0, maxval).astype(int)
        
        #In QAM modulation used, lsb's map to imaginary part.
        wordstream = np.array((imagwordstream + (maxval+1)*realwordstream),dtype=np.uint8)
        bitstream=np.unpackbits(wordstream, axis=0)
        bitstream=bitstream.reshape((-1,8))
        bitstream=bitstream[:,-int(np.log2(QAM))::]
        bitstream=bitstream.reshape((-1,1))

        if self.par:
            self.queue.put(wordstream)
            self.queue.put(bitstream)
        self._wordstream.Value=wordstream
        self._bitstream.Value=bitstream

