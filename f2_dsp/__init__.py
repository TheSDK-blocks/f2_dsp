# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 20.11.2018 20:30
#Add TheSDK to path. Importing it first adds the rest of the modules
#Simple buffer template
import os
import sys
import numpy as np
import tempfile

from thesdk import *
from verilog import *
from f2_util_classes import * #Iofifosigs are here
from f2_tx_dsp import *
from f2_rx_dsp import *


class f2_dsp(verilog,thesdk):
    @property
    def _classfile(self):
        return os.path.dirname(os.path.realpath(__file__)) + "/"+__name__

    def __init__(self,*arg): 
        self.proplist=[ 'rxmodels', 
                        'Txantennas', 
                        'Txpower', 
                        'Rxantennas', 
                        'Users', 
                        'Disableuser', 
                        'Nbits',   #Receiver ADC
                        'Txbits',  #Transmitter DAC
                        'Channeldir', 
                        'CPUFBMODE', 
                        'DSPmode', 
                        'dsp_decimator_scales',     # Scales for the rx decimator chain
                        'dsp_decimator_cic3shift',  # Left-shift for the decimator cic integrator
                        'dsp_interpolator_scales',     # Scales for the tx interpolator chain
                        'dsp_interpolator_cic3shift',  # Left-shift for the interpolator cic integrator
                        'rx_output_mode',
                        'noisetemp', 
                        'Rs', 
                        'Rs_dsp', 
                        'Hstf', 
                        'ofdmdict' ,
                        'nserdes' ]; 
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
        self.iptr_A=IO()
        self.dsp_decimator_model='py'
        self.dsp_decimator_scales=[1,1,1,1]
        self.dsp_decimator_cic3shift=0
        self.dsp_interpolator_scales=[8,2,2,512]
        self.dsp_interpolator_cic3shift=4
        self.noisetemp=290
        self.Rs=160e6
        self.Rs_dsp=20e6
        #self.Hstf=1                             #Synchronization filter
        self.rx_output_mode=1
        self.nserdes=1
        self.DEBUG= False
        self.par=False
        if len(arg)>=1:
            parent=arg[0]
            self.copy_propval(parent,self.proplist)
            self.parent =parent;

        self.iptr_A.Data=[refptr() for _ in range(self.Rxantennas)]
        self._Z_real_t=[ refptr() for _ in range(self.Txantennas) ]
        self._Z_real_b=[ refptr() for _ in range(self.Txantennas) ]
        self._Z_imag_t=[ refptr() for _ in range(self.Txantennas) ]
        self._Z_imag_b=[ refptr() for _ in range(self.Txantennas) ]

        #Rx and tx refer to serdes lane tx is the transmitter input of the serdes
        self._io_lanes_tx=[ iofifosigs(**{'users':self.Users}) for _ in range(self.nserdes)] #this is an output
        self._io_lanes_rx=[ iofifosigs(**{'users':self.Users}) for _ in range(self.nserdes)] #this is an input

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
        self.rx_dsp.iptr_A=self.iptr_A       
        #This is quick and dirty. Must redefine proper IO structure later on
        # and add the switchbox 
        for i in range(self.nserdes):
            #All serdes tx_s have the same data
            self._io_lanes_tx[i]=self.rx_dsp._io_ofifo
        for i in range(self.nserdes): 
            #All serdes rx_s have the same data
            #Are connceted to tx_input. Ensure to drive only one
            self._io_lanes_rx[i]=self.tx_dsp.iptr_A
        self.DEBUG= False
        if len(arg)>=1:
            parent=arg[0]
            self.copy_propval(parent,self.proplist)
            self.parent =parent;

        self.init()

    def init(self):
        self.def_verilog()
        self._vlogmodulefiles =list(['clkdiv_n_2_4_8.v', 'AsyncResetReg.v'])
        #Here's how we sim't for the tapeout

        self._vlogparameters=dict([ ('g_Rs_high',self.Rs), ('g_Rs_low',self.Rs_dsp), 
            ('g_tx_shift'            , 0),
            ('g_tx_scale0'           , self.dsp_interpolator_scales[0]),
            ('g_tx_scale1'           , self.dsp_interpolator_scales[1]),
            ('g_tx_scale2'           , self.dsp_interpolator_scales[2]),
            ('g_tx_scale3'           , self.dsp_interpolator_scales[3]),
            ('g_tx_cic3shift'        , self.dsp_interpolator_cic3shift),
            ('g_tx_user_spread_mode' , 0),
            ('g_tx_user_sum_mode'    , 0),
            ('g_tx_user_select_index', 0),
            ('g_tx_interpolator_mode', 4),
            ('g_tx_dac_data_mode'    , 6),
            ('g_rx_shift'            , 0),
            ('g_rx_scale0',self.dsp_decimator_scales[0]),  
            ('g_rx_scale1',self.dsp_decimator_scales[1]),  
            ('g_rx_scale2',self.dsp_decimator_scales[2]),  
            ('g_rx_scale3',self.dsp_decimator_scales[3]),
            ('g_rx_cic3shift', self.dsp_decimator_cic3shift),
            ('g_rx_mode',self.rx_dsp.mode), ##Propagates from decimator. Check if this is ok
            ('g_rx_user_index', 0),
            ('g_rx_antenna_index', 0),
            ('g_rx_output_mode', self.rx_output_mode), 
            ('g_rx_input_mode', 0),
            ("g_rx_inv_adc_clk_pol",1),
            ('g_rx_adc_fifo_lut_mode' ,2),
            ("g_lane_refclk_Ndiv",2), #This should be at least 8x bb
            ("g_lane_refclk_shift","0")
            ])

    def run_tx(self):
        if self.model=='py':
            self.tx_dsp.run()
        elif self.model=='sv':
            self.write_infile()
            a=verilog_iofile(self,**{'name':'Z'})
            a.simparam='-g g_io_Z='+a.file
            b=verilog_iofile(self,**{'name':'io_lanes_tx'})
            b.simparam='-g g_io_lanes_tx='+b.file
            self.run_verilog()
            self.read_outfile()
            del self.iofiles

    def run_rx(self):
        if self.model=='py':
            self.rx_dsp.init()
            self.rx_dsp.run()
        elif self.model=='sv':
            self.write_infile()
            #define outfiles
            a=verilog_iofile(self,**{'name':'Z'})
            a.simparam='-g g_io_Z='+a.file
            b=verilog_iofile(self,**{'name':'io_lanes_tx'})
            b.simparam='-g g_io_lanes_tx='+b.file
            self.run_verilog()
            self.read_outfile()
            del self.iofiles

    def write_infile(self):
        for i in range(self.nserdes):
            for k in range(self.Users):
                if i==0 and k==0:
                    indata=self._io_lanes_rx[i].data[k].udata.Value.reshape(-1,1)
                else:
                    indata=np.r_['1',indata,self._io_lanes_rx[i].data[k].udata.Value.reshape(-1,1)]
        #This adds an iofile to self.iiofiles list
        a=verilog_iofile(self,**{'name':'io_lanes_rx','data':indata})
        a.simparam='-g g_io_lanes_rx='+a.file
        a.write()
        indata=None #Clear variable to save memory

        for i in range(self.Rxantennas):
            if i==0:
                indata=self.iptr_A.Data[i].Value.reshape(-1,1)
            else:
                indata=np.r_['1',indata,self.iptr_A.Data[i].Value.reshape(-1,1)]
        #This adds an iofile to self.iiofiles list
        a=verilog_iofile(self,**{'name':'A','data':indata})
        a.simparam='-g g_io_iptr_A='+a.file
        a.write()
        indata=None #Clear variable to save memory

    def read_outfile(self):
        #Handle the ofiles here as you see the best
        a=list(filter(lambda x:x.name=='Z',self.iofiles))[0]
        a.read(**{'dtype':'object'})
        for i in range(self.Txantennas):
            self._Z_real_t[i].Value=a.data[:,i*self.Txantennas+0].astype('str').reshape(-1,1)
            self._Z_real_b[i].Value=a.data[:,i*self.Txantennas+1].astype('int').reshape(-1,1)
            self._Z_imag_t[i].Value=a.data[:,i*self.Txantennas+2].astype('str').reshape(-1,1)
            self._Z_imag_b[i].Value=a.data[:,i*self.Txantennas+3].astype('int').reshape(-1,1)
        a=None
        a=list(filter(lambda x:x.name=='io_lanes_tx',self.iofiles))[0]
        a.read(**{'dtype':'object'})
        fromfile=a.data.astype('int')
        for i in range(self.Users):
            cols=3 # Real, Imag x 4 x 2 Udata discarded
            ## for now Lets take only the lane 0
            if i==0:
                out=np.zeros((fromfile.shape[0],int(fromfile.shape[1]/(2*1))),dtype=complex)
                out[:,i]=(fromfile[:,2*i]+1j*fromfile[:,2*i+1]) 
            else:
                out[:,i]=(fromfile[:,2*i]+1j*fromfile[:,2*i+1])
            maximum=np.amax([np.abs(np.real(out[:,i])), np.abs(np.imag(out[:,i]))])
            str="Output signal range is %i" %(maximum)
            self.print_log({'type':'I', 'msg': str})
        for k in range(self.Users):
            self._io_lanes_tx[0].data[k].udata.Value=out[:,k].reshape((-1,1))
        a=None

        self.distribute_result()

    def distribute_result(self):
        for k in range(self.Users):
            if self.par:
                self.queue.put(self._io_lanes_tx[0].data[k].udata.Value.reshape(-1,1))
                for i in range(self.Txantennas):
                    self.queue.put(self._Z_real_t[i].Value.reshape(-1,1)) 
                    self.queue.put(self._Z_real_b[i].Value.reshape(-1,1))
                    self.queue.put(self._Z_imag_t[i].Value.reshape(-1,1))
                    self.queue.put(self._Z_imag_b[i].Value.reshape(-1,1))

if __name__=="__main__":
    import matplotlib.pyplot as plt
    from  f2_dsp import *
    t=thesdk()
    t.print_log({'type':'I', 'msg': "This is a testing template. Enjoy"})
