# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 15.08.2018 19:37
#Add TheSDK to path. Importing it first adds the rest of the modules
#Simple buffer template
import os
import sys
if not (os.path.abspath('../../thesdk') in sys.path):
    sys.path.append(os.path.abspath('../../thesdk'))

import numpy as np
import tempfile

from thesdk import *
from verilog import *
from f2_util_classes import *
from f2_tx_dsp import *
from f2_rx_dsp import *


class f2_dsp(verilog,thesdk):
    @property
    def _classfile(self):
        return os.path.dirname(os.path.realpath(__file__)) + "/"+__name__

    def __init__(self,*arg): 
        self.proplist=[ 'rxmodels', 'Txantennas', 'Txpower', 'Rxantennas', 'Users', 'Disableuser', 'Nbits', 'Txbits', 'Channeldir', 'CPUFBMODE', 'DSPmode', 'dsp_decimator_model', 'dsp_decimator_scales', 'noisetemp', 'Rs', 'Rs_dsp', 'Hstf', 'ofdmdict' ,'nserdes' ]; 
        self.rxmodels=[]
        #Signals should be in form s(user,time,Txantenna)
        self.Txantennas=4                       #All the antennas process the same data
        self.Txpower=30                         #Output power per antenna in dBm
        self.Rxantennas=4
        self.Users=2
        self.model='py'
        self.queue= []                          #by default, no parallel processing

        self.Disableuser=[]
        self.Nbits=10  #ADC bits
        self.Txbits=9  #DAC bits
        self.Channeldir='Uplink'
        self.iptr_A=refptr()
        self.dsp_decimator_model='py'
        self.dsp_decimator_scales=[1,1,1,1]
        self.noisetemp=290
        self.Rs=160e6
        self.Rs_dsp=20e6
        self.Hstf=1                             #Synchronization filter
        self.rx_dsp=[]
        self.tx_dsp=[]
        self.nserdes=1
        self.DEBUG= False
        if len(arg)>=1:
            parent=arg[0]
            self.copy_propval(parent,self.proplist)
            self.parent =parent;

        self.iptr_A.Value=[refptr() for _ in range(self.Rxantennas)]
        self._Z_real_t=[ refptr() for _ in range(self.Txantennas) ]
        self._Z_real_b=[ refptr() for _ in range(self.Txantennas) ]
        self._Z_imag_t=[ refptr() for _ in range(self.Txantennas) ]
        self._Z_imag_b=[ refptr() for _ in range(self.Txantennas) ]

        #Rx and tx refer to serdes lane tx is the transmitter input of the serdes
        self._io_lanes_tx=[ iofifosigs(**{'users':self.Users}) for _ in range(self.nserdes)] #this is an output
        self._io_lanes_rx=[ iofifosigs(**{'users':self.Users}) for _ in range(self.nserdes)] #this is an input
        self.init()
    def init(self):
        #This is the definition for the Transceiver system
        # Aim is to make it resemple the actual circuit as closely as possible

        #Tx
        self.tx_dsp=f2_tx_dsp(self)

        #Antenna outputs
        self._Z_real_t=self.tx_dsp._Z_real_t
        self._Z_real_b=self.tx_dsp._Z_real_b
        self._Z_imag_t=self.tx_dsp._Z_imag_t
        self._Z_imag_b=self.tx_dsp._Z_imag_b

        # Rx 
        self.rx_dsp=f2_rx_dsp(self)
        self.iptr_A=self.rx_dsp.iptr_A

        #This is quick and dirty. Must redefine proper IO structure later on
        # and add the switchbox 
        for i in range(self.nserdes):
            #All serdes tx_s have the same data
            self._io_lanes_tx[i]=self.rx_dsp._io_ofifo
        for i in range(self.nserdes): 
            #All serdes rx_s have the same data
            #Are connceted to tx_input. Ensure to drive only one
            self._io_lanes_rx[i]=self.tx_dsp.iptr_A

    def run_tx(self):
        if self.model=='py':
            if self.tx_dsp.model=='sv':
                self.tx_dsp.init()
                self.tx_dsp.run()
            else:    
                self.print_log({'type':'F', 'msg': "Python model not available for TX DSP"})
        elif self.mode=='sv':
            self.write_infile_tx()
            self.run_verilog()
            self.read_outfile_tx()

    def run_rx(self):
        if self.model=='py':
            self.rx_dsp.init()
            self.rx_dsp.run()
        elif self.mode=='sv':
            self.write_infile_rx()
            self.run_verilog()
            self.read_outfile_rx()

    def write_infile(self):
        rndpart=os.path.basename(tempfile.mkstemp()[1])
        if self.model=='sv':
            self._infile=self._vlogsimpath +'/A_' + rndpart +'.txt'
            self._outfile=self._vlogsimpath +'/Z_' + rndpart +'.txt'
        elif self.model=='vhdl':
            self._infile=self._vhdlsimpath +'/A_' + rndpart +'.txt'
            self._outfile=self._vhdlsimpath +'/Z_' + rndpart +'.txt'
        else:
            pass
        try:
          os.remove(self._infile)
        except:
          pass
        fid=open(self._infile,'wb')
        np.savetxt(fid,np.transpose(self.iptr_A.Value),fmt='%.0f')
        #np.savetxt(fid,self.iptr_A.Value.reshape(-1,1).view(float),fmt='%i', delimiter='\t')
        fid.close()

    def read_outfile(self):
        fid=open(self._outfile,'r')
        out = np.transpose(np.loadtxt(fid))
        #out = np.loadtxt(fid,dtype=complex)
        #Of course it does not work symmetrically with savetxt
        #out=(out[:,0]+1j*out[:,1]).reshape(-1,1) 
        fid.close()
        os.remove(self._outfile)
        if self.par:
            self.queue.put(out)
        self._Z.Value=out

if __name__=="__main__":
    import matplotlib.pyplot as plt
    from  f2_dsp import *
    t=thesdk()
    t.print_log({'type':'I', 'msg': "This is a testing template. Enjoy"})
