# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 19.01.2018 14:15
import numpy as np
import scipy.signal as sig
import tempfile
import subprocess
import shlex
import time

from refptr import *
from thesdk import *
from f2_decimator import *
import signal_generator_802_11n as sg80211n


#Simple buffer template
class f2_dsp(thesdk):
    def __init__(self,*arg): 
        self.proplist = [ 'Rs', 'Rs_dsp', 'Hstf', 'Hltf', 'Users', 'DSPmode' ];    #properties that can be propagated from parent
        self.Rs = 160e6;                 # sampling frequency
        self.Rs_dsp=20e6
        self.Users=1
        self.Antennaindex=0                 #This is to control the sync and channel estimation
        self.Hstf=1                      #filters for sybol sync
        self.Hltf=1
        self.iptr_A = refptr();
        self.iptr_reception_vect=refptr()
        self.model='py';                 #can be set externally, but is not propagated
        self.dsp_decimator_model='sv'
        self.rtldiscard=50
        self.DSPmode='cpu';              # [ 'local' | 'cpu' ]  
        self.par= False                  #by default, no parallel processing
        self.queue= []                   #by default, no parallel processing
        self._decimated=refptr()         #signals sampled at rs_dsp
        self._delayed=refptr()
        self._sync_index=refptr()        #signal index for the symbol synchronization
        self._ch_index=[]                #Index vector for the channle synchronization (internal) 
        self._channel_est=refptr()
        self._channel_corr=refptr()
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
        if len(arg) >=2:
            self.Antennaindex=arg[1]
            self.print_log({'type':'I','msg':"Setting Antenna index %s to %i" %(self, self.Antennaindex)})
        self._channel_est.Value=[refptr() for i in range(self.Users)]
        self._channel_corr.Value=[refptr() for i in range(self.Users)]
        self._symbols.Value=[refptr() for i in range(self.Users)]
        self._wordstream.Value=[refptr() for i in range(self.Users)]
        self._bitstream.Value=[refptr() for i in range(self.Users)]
        self.iptr_reception_vect.Value=[refptr() for i in range(self.Users)]

    def init(self):
        #Add sublocks
        self.decimator=f2_decimator()
        self.decimator.Rs_high=self.Rs
        self.decimator.Rs_low=self.Rs_dsp
        self.decimator.model=self.dsp_decimator_model
        self.decimator.iptr_A=self.iptr_A
        self.decimator.scales=[1,2^10,1,1]
        self.decimator.init()


    def get_channel_estimate(self,*arg):
        if len(arg)>0:
            self.par=True      #flag for parallel processing
            self.queue=arg[0]  #multiprocessing.queue as the first argument
     
        self.decimate_input()
        self.delay_input()
        self.symbol_sync()
        self.estimate_channel()

    def receive_data(self,*arg):
        if len(arg)>0:
            self.par=True      #flag for parallel processing
            self.queue=arg[0]  #multiprocessing.queue as the first argument
     
        self.receive_symbols()
        self.extract_data()


    def run(self,*arg):
        if len(arg)>0:
            self.par=True      #flag for parallel processing
            self.queue=arg[0]  #multiprocessing.queue as the first argument

        if self.model=='py':
            self.get_channel_estimate()
            self.receive_data()

        else: 
            self.print_log({'type':'I', 'msg':'Copy the verilog simulations setup from f2_decimator'})

    def decimate_input(self):
        self.decimator.run()
        #Simple energy detector
        discard=np.nonzero(np.abs(self.decimator._Z.Value.reshape((-1,1)))>0)[0][0]
        
        decimated=self.decimator._Z.Value.reshape((-1,1))[self.rtldiscard::,0].reshape((-1,1))
        self.print_log({'type':'W', 'msg':'Discarded %i zero samples to remove possibble initial transients in symbol sync.' %(self.rtldiscard)})
        if self.par:
           self.queue.put(decimated)
        self._decimated.Value=decimated
        print(decimated.shape)
    
    def delay_input(self):
        #dummy filter to delay the payload signal in paralled with symbol sync
        dfil1=np.zeros(self.Hltf.shape)
        dfil1[-1,0]=1
        delayed=sig.convolve(self._decimated.Value, dfil1, mode='full')

        #dummy filter to delay the payload signal
        dfil2=np.zeros((6,1))
        dfil2[-1,0]=1
        delayed=sig.convolve(delayed, dfil2, mode='full')
        if self.par:
           self.queue.put(delayed)

        self._delayed.Value=delayed
        self.print_log({'type':'D', 'msg': "internal delayed.shape for testing for user %i is %s" %(self.Antennaindex, self._delayed.Value.shape)})
        

    def symbol_sync(self):
        #matched filtering for short and long sequences and squaring for energy
        #scale according to l2 norm
        framelen=sg80211n.ofdm64dict_noguardband['framelen']
        CPlen=sg80211n.ofdm64dict_noguardband['CPlen']

        #the maximum of this is sum of squares squared
        matchedshort=np.abs(sig.convolve(self._decimated.Value, self.Hstf, mode='full'))**2
        matchedlong=(np.sum(np.abs(self.Hstf)**2)/np.sum(np.abs(self.Hltf)**2)*np.abs(sig.convolve(self._decimated.Value, self.Hltf, mode='full')))**2

        #filter for energy filtering (average of 6 samples)
        efil=np.ones((6,1))
        sshort=sig.convolve(matchedshort,efil,mode='full')
        slong=sig.convolve(matchedlong,efil,mode='full')

        #sum of the 4 past spikes with separation of 16 samples
        sfil=np.zeros((65,1))
        sfil[16:65:16]=1
        sspikes_short=sig.convolve(sshort,sfil,mode='full')/np.sum(np.abs(sfil))
        
        #compensate the matched long for the filter delays of the short
        delay=len(sspikes_short)-len(slong)
        slong=np.r_['0',slong, np.zeros((delay,1))]
        sspikes_sum=slong+sspikes_short
        self.sspikes_short=sspikes_short

        #find the frame start
        usercount=0
        #found=False
        i=0
        smaxprev=0
        indmaxprev=0
        ch_index=np.zeros((self.Users,1))
        #ch_index_est=np.zeros((self.Users,1))
        while usercount< self.Users and i+16<= len(sspikes_sum):
            self.print_log({'type':'D', 'msg': "Syncing user %i" %(usercount)})
            testwin=(sspikes_sum[i:i+16])
            self.print_log({'type':'D', 'msg': "Testwin is %s" %(testwin)})
            smax=np.max(testwin)
            indmax=i+np.argmax(testwin,axis=0)
            
            if smax != sspikes_sum[indmax]:
                self.print_log({'type':'F', 'msg': "Something wrong with the symbol boundary"})

            if smax < 0.85 * smaxprev:
                #found = True
                smax=smaxprev
                indmax=indmaxprev
                self.print_log({'type':'I', 'msg': "Found the symbol boundary at sample %i" %(indmax)})
                
                #this is an estimate of the start of the channel estimation for 
                # a user.
                ch_index[usercount]=int(indmaxprev)+int(efil.shape[0]/2)
                #this is to prevent accidental sync to long sequence of the current/short sequence of the next user. 
                smaxprev=0 #reset the spike maximum Value
                usercount+=1
                #This is suffient jump to avoid incorrect sync
                i+=256
            else:
                smaxprev=smax
                indmaxprev=indmax
                i+=16
        #all user sync found. calculate average
        c=int(np.mean(ch_index-np.arange(self.Users)*(4*(framelen+CPlen))))
        ch_index=np.arange(self.Users)*4*(framelen+CPlen)+c
        ch_index.shape=(-1,1)
        #common sync index calculated from the last channel estimation index
        sync_index=int(ch_index[-1,0]+2*len(sg80211n.PLPCsyn_long))

        if self.par:
            self.print_log({'type':'D', 'msg': "putting sync_index for RX path %i: %s" %(self.Antennaindex,sync_index)})
            self.queue.put(int(sync_index))
            self.queue.put(sspikes_short)
            self.queue.put(sspikes_sum)

        #keep the sync indexes as pointers, even though they are not needed outside the dsp
        self._ch_index=ch_index
        self._Frame_sync_short.Value=sspikes_short
        self._Frame_sync_long.Value=sspikes_sum
        self.print_log({'type':'D','msg': "sync_index is %s" %(sync_index)})
        self._sync_index.Value=sync_index
        self.print_log({'type':'I', 'msg': "start of the RX path %i data is at %i" %(self.Antennaindex,self._sync_index.Value)})

    def estimate_channel(self):
        for i in range(self.Users):
            #Offset is used to position the sampling istant in the middle of cyclic prefixes
            #of the payload symbols
            #If delay is not correct, it is visible as phase offset and is compensated by
            #channel egualization, as long as the delay offset is the same for long sequence and
            #payload frames
            offset=int(sg80211n.TGI2/2)+int(sg80211n.ofdm64dict_noguardband['CPlen']/2)
            #offset=int(sg80211n.TGI2/2)+int(sg80211n.ofdm64dict_noguardband['CPlen']/2-3)
            long_sequence=self._delayed.Value[self._ch_index[i,0]+offset:self._ch_index[i,0]+offset+128,0].T
            long_seq_freq=long_sequence.reshape((-1,64))
            long_seq_freq=np.sum(long_seq_freq,axis=0)
            long_seq_freq=np.fft.fft(long_seq_freq)

            #Map the negative frequencies to the beginning of the array
            long_seq_freq=long_seq_freq[sg80211n.Freqmap]

            #Estimate the channel
            channel_est=np.multiply(long_seq_freq,sg80211n.PLPCsyn_long.T)
            self.print_log({'type':'D', 'msg':"Channel estimate in dB is %s " %(20*np.log10(np.abs(channel_est)/np.max(np.abs(channel_est))))}  )
            channel_corr=np.zeros_like(channel_est)

            data_loc=sg80211n.ofdm64dict_withguardband['data_loc']
            pilot_loc=sg80211n.ofdm64dict_withguardband['pilot_loc']
            data_and_pilot_loc=np.sort(np.r_[data_loc, pilot_loc])

            channel_corr[0,data_and_pilot_loc+32]=np.conj(channel_est[0,data_and_pilot_loc+32])
            self.print_log({'type':'D', 'msg':"Corrected channel  is %s " %(1/np.conj(channel_corr)*channel_est)}  )

            if self.par:
                self.print_log({'type':'D', 'msg':"Putting estimates in RX path %s for user %s" %(self.Antennaindex, i)}  )
                self.queue.put(channel_est)
                self.queue.put(channel_corr)

            self.print_log({'type':'D', 'msg':"Assigning pointers for user %s" %(i)}  )
            self._channel_est.Value[i].Value=channel_est
            self._channel_corr.Value[i].Value=channel_corr

    def receive_symbols(self):
        self.print_log({'type':'D', 'msg':"Receiving symbols"}  )
        #Ofdm manipulations start here
        #Start the OFDM demodulation here
        offset=int(sg80211n.TGI2/2)+int(sg80211n.ofdm64dict_noguardband['CPlen']/2)
        data_loc=sg80211n.ofdm64dict_withguardband['data_loc']
        pilot_loc=sg80211n.ofdm64dict_withguardband['pilot_loc']
        data_and_pilot_loc=np.sort(np.r_[data_loc, pilot_loc])
        cyclicsymlen=int(sg80211n.ofdm64dict_noguardband['framelen']+sg80211n.ofdm64dict_noguardband['CPlen'])

        if self.DSPmode== 'local':
            self.print_log({'type':'D', 'msg':"Local Sync index is %s " %(self._sync_index.Value)}  )
            payload=self._delayed.Value[self._sync_index.Value+offset::]
        elif self.DSPmode== 'cpu':
            self.print_log({'type':'D', 'msg':"Local Sync index id %s " %(self._sync_index.Value)}  )
            payload=self._delayed.Value[self._sync_index.Value+offset::]
        else:
            self.print_log({'type':'F', 'msg':"DSPmode %s not supported" })

        length=int(np.floor(payload.shape[0]/cyclicsymlen)*(cyclicsymlen))
        payload=payload[0:length,0]
        payload.shape=(1,-1)
        payload=payload.reshape((-1,cyclicsymlen))
        #Strip the cyclic prefix
        #All the users in the receiver branch have the same sync Value
        payload=payload[:,sg80211n.ofdm64dict_noguardband['CPlen']::]
        demod=np.fft.fft(payload,axis=1)
        demod=demod[:,sg80211n.Freqmap]

        for i in range(self.Users):
            if self.DSPmode== 'local':
                corr_value=1/self._channel_est.Value[i].Value
                corr_mat=np.ones((demod.shape[0],1))@corr_value
                self.print_log({'type':'D', 'msg':"Corrected channel for user %s is %s " %(i, corr_value*self._channel_est.Value[i].Value)}  )
            elif self.DSPmode== 'cpu':
                corr_mat=np.ones((demod.shape[0],1))@self.iptr_reception_vect.Value[i].Value
                #corr_value=self.iptr_reception_vect.Value
                #corr_mat=np.ones((demod.shape[0],1))@corr_value
                self.print_log({'type':'D', 'msg':"Corrected channel for user %s is %s " %(i, self.iptr_reception_vect.Value[i].Value*self._channel_est.Value[i].Value)}  )
            else:
                self.print_log({'type':'F', 'msg':"DSPmode %s not supported" })

            udemod=np.multiply(demod,corr_mat)
            udemod=udemod[:,data_and_pilot_loc+int(sg80211n.ofdm64dict_noguardband['framelen']/2)]
            udemod=udemod.reshape(-1,1)

            if self.par:
                self.queue.put(udemod)
            #Each user have their own symbol stram output.
            self._symbols.Value[i].Value=udemod
            self.print_log({'type':'D', 'msg':"Udemod symbols shape is (%s,%s) " %(self._symbols.Value[i].Value.shape)})

    def extract_data(self):
        QAM=16
        maxval=int(np.sqrt(QAM))-1
        #Scaling for quantization
        conststd=np.sqrt(np.sum(np.arange(1,maxval+1,2)**2))/len(np.arange(1,maxval+1,2))
       
        for i in range(self.Users):
            #Shift to positive and quantize
            normalized=self._symbols.Value[i].Value/np.std(self._symbols.Value[i].Value)*conststd+maxval/2*(1+1j)
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
                self.print_log({'type':'D', 'msg':"Putting streams"})
                self.queue.put(wordstream)
                self.queue.put(bitstream)
            self._wordstream.Value[i].Value=wordstream
            self._bitstream.Value[i].Value=bitstream

