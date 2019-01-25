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
from verilog.testbench import *
from verilog.testbench import testbench as vtb 
from verilog.connector import intend

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
        self.Users=16
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
        self.nserdes=2
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
        #This gets updated every time you add an iofile
        self._iofile_bundle=Bundle()
        #Adds filea to bundle
        #Letter variables are just for shorthand notation
        a=verilog_iofile(self,name='Z')
        a.simparam='-g g_io_Z='+a.file
        b=verilog_iofile(self,name='io_lanes_tx')
        b.simparam='-g g_io_lanes_tx='+b.file
        c=verilog_iofile(self,name='io_lanes_rx',dir='in')
        c.simparam='-g g_io_lanes_rx='+c.file
        d=verilog_iofile(self,name='A',dir='in')
        d.simparam='-g g_io_iptr_A='+d.file
        e=verilog_iofile(self,name='serdestest_write',dir='in',iotype='ctrl')
        e.simparam='-g g_serdestest_write='+e.file

        self.vlogmodulefiles =list(['clkdiv_n_2_4_8.v', 'AsyncResetReg.v'])
        self.vlogparameters=dict([ ('g_Rs_high',self.Rs), ('g_Rs_low',self.Rs_dsp), 
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
        
        self.define_testbench()

    def run_tx(self):
        if self.model=='py':
            self.tx_dsp.run()
        elif self.model=='sv':
            self.write_infile()
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
        self._iofile_bundle.Members['io_lanes_rx'].data=indata
        self._iofile_bundle.Members['io_lanes_rx'].write()
        indata=None #Clear variable to save memory

        for i in range(self.Rxantennas):
            if i==0:
                indata=self.iptr_A.Data[i].Value.reshape(-1,1)
            else:
                indata=np.r_['1',indata,self.iptr_A.Data[i].Value.reshape(-1,1)]
        #This adds an iofile to self.iofiles list
        self._iofile_bundle.Members['A'].data=indata
        self._iofile_bundle.Members['A'].write()
        indata=None #Clear variable to save memory

    def read_outfile(self):
        #Handle the ofiles here as you see the best
        a=self._iofile_bundle.Members['Z']
        a.read(dtype='object')
        for i in range(self.Txantennas):
            self._Z_real_t[i].Value=a.data[:,i*self.Txantennas+0].astype('str').reshape(-1,1)
            self._Z_real_b[i].Value=a.data[:,i*self.Txantennas+1].astype('int').reshape(-1,1)
            self._Z_imag_t[i].Value=a.data[:,i*self.Txantennas+2].astype('str').reshape(-1,1)
            self._Z_imag_b[i].Value=a.data[:,i*self.Txantennas+3].astype('int').reshape(-1,1)
        a=None
        a=self._iofile_bundle.Members['io_lanes_tx']
        a.read(dtype='object')
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

    def define_testbench(self):
        self.tb=vtb(self)
        self.tb.parameters=self.vlogparameters
        self.tb.iofiles=self.iofile_bundle
        self.tb.file=self.vlogtbsrc

        clockdivider=verilog_module(file=self.vlogsrcpath+'/clkdiv_n_2_4_8.v', 
                instname='clockdivider')
        #This is a bundle of connectors connected to clockdivider ios
        clockdivider.io_signals.mv(fro='reset',to='reset_clock_div')
        clockdivider.io_signals.mv(fro='io_Ndiv',to='lane_refclock_Ndiv')
        clockdivider.io_signals.mv(fro='io_shift',to='lane_refclock_shift')
        clockdivider.io_signals.mv(fro='io_reset_clk',to='lane_refclock_shift')
        clockdivider.io_signals.mv(fro='io_clkpn',to='lane_clockref')
        
        self.tb.connectors.update(bundle=clockdivider.io_signals.Members)
        self.tb.connectors.update(bundle=self.tb.dut_instance.io_signals.Members)

        self.tb.connectors.new(name='reset_loop', cls='reg')
        self.tb.connectors.new(name='lane_clockRef', cls='reg')

        self.tb.connectors.connect(match=r"io_ctrl_and_clocks_.*_controls_.?_reset_loop",connect='reset_loop')
        self.tb.connectors.connect(match=r"io_ctrl_and_clocks_(dac|adc)_clocks_.?",connect='clock')
        self.tb.connectors.connect(match=r"io_lanes_tx_enq_clock",connect='lane_clockRef')

        #Init the signals connected to the dut input
        for name, val in self.tb.dut_instance.ios.Members.items():
            if val.cls=='input':
                val.connect.init='\'b0'
        #Init the signals connected to the clockdivider input
        for name, val in clockdivider.ios.Members.items():
            if val.cls=='input':
                val.connect.init='\'b0'

        #IO file connector definitions
        a=self.iofile_bundle.Members['Z']
        ionames=[]
        for count in range(self.Rxantennas): 
            ionames.append('io_Z_%s_real_t' %(count)) 
            ionames.append('io_Z_%s_real_b' %(count)) 
            ionames.append('io_Z_%s_imag_t' %(count)) 
            ionames.append('io_Z_%s_imag_b' %(count))
        a.verilog_connectors=self.tb.connectors.list(names=ionames)
        #Format for thermobits is %b
        for val in a.verilog_connectors:
            if re.match(r".*real_t",val.name):
                val.ioformat="%b"
        
        a=self.iofile_bundle.Members['io_lanes_tx']
        ionames=[]
        for user in range(self.Users): 
            ionames.append('io_lanes_tx_0_bits_data_%s_udata_real' %(user)) 
            ionames.append('io_lanes_tx_0_bits_data_%s_udata_imag' %(user)) 
        a.verilog_connectors=self.tb.connectors.list(names=ionames)

        ionames=[]
        a=self.iofile_bundle.Members['A']
        for count in range(self.Rxantennas): 
            ionames.append('io_iptr_A_%s_real' %(count)) 
            ionames.append('io_iptr_A_%s_imag' %(count))
        a.verilog_connectors=self.tb.connectors.list(names=ionames)

        a=self.iofile_bundle.Members['io_lanes_rx']
        ionames=[]
        for serdes in range(self.nserdes):
            for user in range(self.Users): 
                ionames.append('io_lanes_rx_%s_bits_data_%s_udata_real' %(serdes,user)) 
                ionames.append('io_lanes_rx_%s_bits_data_%s_udata_imag' %(serdes,user)) 
        a.verilog_connectors=self.tb.connectors.list(names=ionames)

        a=self.iofile_bundle.Members['serdestest_write']
        ionames=[]
        ionames.append('io_ctrl_and_clocks_serdestest_scan_write_mode') 
        ionames.append('io_ctrl_and_clocks_serdestest_scan_write_address') 
        ionames.append('io_ctrl_and_clocks_serdestest_scan_write_en') 
        ionames.append('io_ctrl_and_clocks_serdestest_scan_write_value_rxindex') 
        for user in range(self.Users): 
            ionames.append('io_ctrl_and_clocks_serdestest_scan_write_value_data_%s_udata_real' %(user)) 
            ionames.append('io_ctrl_and_clocks_serdestest_scan_write_value_data_%s_udata_imag' %(user)) 
        a.verilog_connectors=self.tb.connectors.list(names=ionames)
        
        #Here start the testbench contents
        self.tb.contents="""
//timescale 1ps this should probably be a global model parameter 
parameter integer c_Ts=1/(g_Rs_high*1e-12);
parameter tx_c_ratio=g_Rs_high/(8*g_Rs_low);
parameter rx_c_ratio=g_Rs_high/(8*g_Rs_low);
parameter RESET_TIME = 128*c_Ts; // initially 16
"""+ self.tb.connector_definitions+self.tb.assignments(
        matchlist=[r"io_ctrl_and_clocks_(dac|adc)_clocks_.?", 
                    r"io_ctrl_and_clocks_.*_controls_.?_reset_loop", 
                    r"io_lanes_tx_enq_clock"])+self.tb.iofile_definitions+"""


integer memaddrcount;
integer initdone, rxdone, txdone;

//Initializations
initial clock = 1'b0;
initial reset = 1'b0;

//Clock definitions
always #(c_Ts/2.0) clock = !clock ;

//Tx_io
always @(posedge io_ctrl_and_clocks_dac_clocks_0 ) begin 
    //Print only valid values 
    if ((initdone==1) && txdone==0 && \n"""+self._iofile_bundle.Members['Z'].verilog_io_condition + """
    ) begin \n"""+ self._iofile_bundle.Members['Z'].verilog_io+"""
     end
end
//Rx_io
always @(posedge lane_clockRef ) begin 
//Mimic the reading to lanes
    //Print only valid values 
    if ((io_lanes_tx_0_valid==1) &&  (initdone==1) && rxdone==0 &&
    """+self._iofile_bundle.Members['io_lanes_tx'].verilog_io_condition + """
) begin 
"""+self._iofile_bundle.Members['io_lanes_tx'].verilog_io + """ 
    end
end
                        
//Clock divider model\n""" + clockdivider.instance + """//DUT definition
"""+self.tb.dut_instance.instance+"""
//Initial values 
initial #0 begin\n""" + self.tb.connector_inits(level=1) + """

    //This should be transferred to control file
    #(RESET_TIME)
    initdone=0;
    txdone=0;
    rxdone=0;
    reset_clock_div=0;
    io_ctrl_and_clocks_tx_reset_clkdiv=0;
    io_ctrl_and_clocks_rx_reset_clkdiv=0;
    lane_refclk_reset=0;
    io_ctrl_and_clocks_reset_dacfifo=0;
    io_ctrl_and_clocks_reset_outfifo=0;
    io_ctrl_and_clocks_reset_infifo=0;
    #(2*RESET_TIME)
    reset=0;
    #(16*RESET_TIME)
    reset_loop=0;
    io_ctrl_and_clocks_reset_adcfifo=0;
    memaddrcount=0;
//Init the LUT
    while (memaddrcount<2**9) begin
       //This is really controlled by Scan, but we do not have scan model 
       @(posedge clkp8n) 
       io_ctrl_and_clocks_dac_lut_write_en_0<=1;
       io_ctrl_and_clocks_dac_lut_write_en_1<=1;
       io_ctrl_and_clocks_dac_lut_write_en_2<=1;
       io_ctrl_and_clocks_dac_lut_write_en_3<=1;
       io_ctrl_and_clocks_dac_lut_write_addr_0<=memaddrcount;
       io_ctrl_and_clocks_dac_lut_write_addr_1<=memaddrcount;
       io_ctrl_and_clocks_dac_lut_write_addr_2<=memaddrcount;
       io_ctrl_and_clocks_dac_lut_write_addr_3<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_en<=1;
       io_ctrl_and_clocks_adc_lut_write_addr<=memaddrcount;
       if (memaddrcount < 2**8) begin
          io_ctrl_and_clocks_dac_lut_write_vals_0_real<=memaddrcount+2**8; 
          io_ctrl_and_clocks_dac_lut_write_vals_1_real<=memaddrcount+2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_2_real<=memaddrcount+2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_3_real<=memaddrcount+2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_0_imag<=memaddrcount+2**8; 
          io_ctrl_and_clocks_dac_lut_write_vals_1_imag<=memaddrcount+2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_2_imag<=memaddrcount+2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_3_imag<=memaddrcount+2**8;
       end

       else begin
          io_ctrl_and_clocks_dac_lut_write_vals_0_real<=memaddrcount-2**8; 
          io_ctrl_and_clocks_dac_lut_write_vals_1_real<=memaddrcount-2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_2_real<=memaddrcount-2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_3_real<=memaddrcount-2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_0_imag<=memaddrcount-2**8; 
          io_ctrl_and_clocks_dac_lut_write_vals_1_imag<=memaddrcount-2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_2_imag<=memaddrcount-2**8;
          io_ctrl_and_clocks_dac_lut_write_vals_3_imag<=memaddrcount-2**8;
        end  
       //ADC ctrl_and_clocks_LUT
       io_ctrl_and_clocks_adc_lut_write_en<=1;
       io_ctrl_and_clocks_adc_lut_write_addr<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_vals_0_real<=memaddrcount; 
       io_ctrl_and_clocks_adc_lut_write_vals_1_real<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_vals_2_real<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_vals_3_real<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_vals_0_imag<=memaddrcount; 
       io_ctrl_and_clocks_adc_lut_write_vals_1_imag<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_vals_2_imag<=memaddrcount;
       io_ctrl_and_clocks_adc_lut_write_vals_3_imag<=memaddrcount;
       @(posedge clkp8n) 
       memaddrcount=memaddrcount+1;
       io_ctrl_and_clocks_dac_lut_write_en_0<=0;
       io_ctrl_and_clocks_dac_lut_write_en_1<=0;
       io_ctrl_and_clocks_dac_lut_write_en_2<=0;
       io_ctrl_and_clocks_dac_lut_write_en_3<=0;
       io_ctrl_and_clocks_adc_lut_write_en<=0;
    end
    io_ctrl_and_clocks_adc_lut_reset<=0;
    initdone<=1;
    fork
     //status_io_lanes_rx=$fgets(dummyline,f_io_lanes_rx);
     while (!$feof(f_io_lanes_rx)) begin
             txdone<=0;
             //Lane output fifo is read by the symrate clock
             @(posedge io_lanes_rx_deq_clock )
             """+ self.iofile_bundle.Members['io_lanes_rx'].verilog_io+"""
            txdone<=1;
        end
        while (!$feof(f_io_iptr_A)) begin
            rxdone<=0;
            @(posedge io_ctrl_and_clocks_adc_clocks_0 )
            """+self.iofile_bundle.Members['A'].verilog_io+"""
           rxdone<=1;
        end"""+self.iofile_bundle.Members['serdestest_write'].verilog_io+"""
    end
    join
    """+self.tb.iofile_close+"""
    $finish;
end"""

if __name__=="__main__":
    import textwrap
    from verilog import *
    from verilog.testbench import *
    from f2_dsp import *
    t=f2_dsp()
    #print(t.tb.dut_instance.ios.Members['clock'].connect.name)
    #for name, signal in t.tb.dut_instance.ios.Members.items():
    #    print('Name is %s, width is %s, and dir is %s, will be connected to to %s' %(signal.name, signal.width,signal.cls, signal.connect.name))
    #for name, signal in t.tb.dut_instance.io_signals.Members.items():
    #    print('Name is %s, width is %s, and type is %s, will be connected to %s of cls %s' %(signal.name, signal.width,signal.cls, signal.connect.Members[signal.name].name, signal.connect.Members[signal.name].cls))
    #print(t.tb.instance)

    #print(t.tb.dut_instance.ios[0].connect)
    #t.tb.dut_instance.ios[0].connect='Gerbil'
    #print(t.tb.dut_instance.ios[0].connect)
    #print(t.tb.parameters)
    #print(t.tb.connector_inits())
    print(t.tb.definition)
    t.tb.export(force=True)
    #print(t.tb.connector_definitions)

