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
        self.scan=IO()
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
        self.scan=IO()
        self.scan.Data=Bundle()

        self.iptr_A.Data=[IO() for _ in range(self.Rxantennas)]
        self._Z_real_t=[ IO() for _ in range(self.Txantennas) ]
        self._Z_real_b=[ IO() for _ in range(self.Txantennas) ]
        self._Z_imag_t=[ IO() for _ in range(self.Txantennas) ]
        self._Z_imag_b=[ IO() for _ in range(self.Txantennas) ]

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
        self.iofile_bundle=Bundle()
        #Adds files to bundle
        _=verilog_iofile(self,name='Z')
        _=verilog_iofile(self,name='io_lanes_tx')
        _=verilog_iofile(self,name='io_lanes_rx',dir='in')
        _=verilog_iofile(self,name='A',dir='in')
        _=verilog_iofile(self,name='serdestest_write',dir='in',iotype='ctrl')

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


    def run_tx(self):
        if self.model=='py':
            self.tx_dsp.run()
        elif self.model=='sv':
            for name, val in self.scan.Data.Members.items():
                # These files are created under f2_scan_controller
                self.iofile_bundle.Members[name]=val
            self.define_testbench()
            self.tb.export(force=True)
            self.write_infile()
            self.run_verilog()
            self.read_outfile()
            del self.iofile_bundle

    def run_rx(self):
        if self.model=='py':
            self.rx_dsp.init()
            self.rx_dsp.run()
        elif self.model=='sv':
            for name, val in self.scan.Data.Members.items():
                self.iofile_bundle.Members[name]=val
            print(self.iofile_bundle.Members)
            self.define_testbench()
            self.tb.export(force=True)
            self.write_infile()
            self.run_verilog()
            self.read_outfile()
            del self.iofile_bundle

    def write_infile(self):
        for i in range(self.nserdes):
            for k in range(self.Users):
                if i==0 and k==0:
                    indata=self._io_lanes_rx[i].data[k].udata.Data.reshape(-1,1)
                else:
                    indata=np.r_['1',indata,self._io_lanes_rx[i].data[k].udata.Data.reshape(-1,1)]
        #This adds an iofile to self.iiofiles list
        self.iofile_bundle.Members['io_lanes_rx'].data=indata
        self.iofile_bundle.Members['io_lanes_rx'].write()
        indata=None #Clear variable to save memory
        self.iofile_bundle.Members['serdestest_write'].data=np.array([0, 0]).reshape(-1,2)
        self.iofile_bundle.Members['serdestest_write'].write()
        self.iofile_bundle.Members['reset_sequence'].write()


        for i in range(self.Rxantennas):
            if i==0:
                indata=self.iptr_A.Data[i].Data.reshape(-1,1)
            else:
                indata=np.r_['1',indata,self.iptr_A.Data[i].Data.reshape(-1,1)]
        #This adds an iofile to self.iofiles list
        self.iofile_bundle.Members['A'].data=indata
        self.iofile_bundle.Members['A'].write()
        indata=None #Clear variable to save memory

    def read_outfile(self):
        #Handle the ofiles here as you see the best
        a=self.iofile_bundle.Members['Z']
        a.read(dtype='object')
        for i in range(self.Txantennas):
            self._Z_real_t[i].Data=a.data[:,i*self.Txantennas+0].astype('str').reshape(-1,1)
            self._Z_real_b[i].Data=a.data[:,i*self.Txantennas+1].astype('int').reshape(-1,1)
            self._Z_imag_t[i].Data=a.data[:,i*self.Txantennas+2].astype('str').reshape(-1,1)
            self._Z_imag_b[i].Data=a.data[:,i*self.Txantennas+3].astype('int').reshape(-1,1)
        a=None
        a=self.iofile_bundle.Members['io_lanes_tx']
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
            self.print_log(type='I', msg=str)
        for k in range(self.Users):
            self._io_lanes_tx[0].data[k].udata.Data=out[:,k].reshape((-1,1))
        a=None

        self.distribute_result()

    def distribute_result(self):
        for k in range(self.Users):
            if self.par:
                self.queue.put(self._io_lanes_tx[0].data[k].udata.Data.reshape(-1,1))
                for i in range(self.Txantennas):
                    self.queue.put(self._Z_real_t[i].Data.reshape(-1,1))
                    self.queue.put(self._Z_real_b[i].Data.reshape(-1,1))
                    self.queue.put(self._Z_imag_t[i].Data.reshape(-1,1))
                    self.queue.put(self._Z_imag_b[i].Data.reshape(-1,1))

     
    # Define method that generates reset sewunce verilog
    def reset_sequence(self):
        reset_sequence='begin\n'+self.iofile_bundle.Members['reset_sequence'].verilog_io+"""
memaddrcount=0;
//Init the LUT
    while (memaddrcount<2**9) begin
       //This is really controlled by Scan, but we do not have scan model
       @(posedge io_clkp8n)
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
       @(posedge io_clkp8n)
       memaddrcount=memaddrcount+1;
       io_ctrl_and_clocks_dac_lut_write_en_0<=0;
       io_ctrl_and_clocks_dac_lut_write_en_1<=0;
       io_ctrl_and_clocks_dac_lut_write_en_2<=0;
       io_ctrl_and_clocks_dac_lut_write_en_3<=0;
       io_ctrl_and_clocks_adc_lut_write_en<=0;
    end
    io_ctrl_and_clocks_adc_lut_reset<=0;
    initdone<=1; //Flags that we are initiaized
end"""
        return reset_sequence

    # Testbench definition method
    def define_testbench(self):
        #Initialize testbench
        self.tb=vtb(self)

        #Assign verilog simulation parameters to testbench
        self.tb.parameters=self.vlogparameters

        # Copy iofile simulation parametrs to testbench
        for name, val in self.iofile_bundle.Members.items():
            self.tb.parameters.Members.update(val.vlogparam)

        # Define the iofiles of the testbench. '
        # Needed for creating file io routines 
        self.tb.iofiles=self.iofile_bundle

        #Define testbench verilog source file
        self.tb.file=self.vlogtbsrc

        # Create clockdivider instance
        clockdivider=verilog_module(file=self.vlogsrcpath+'/clkdiv_n_2_4_8.v',
                instname='clockdivider')

        # This is a bundle of connectors connected to clockdivider ios
        # Signals/regs connected to clockdivider are automatically availabe
        # Some of them need to be renamed (mv)
        clockdivider.io_signals.mv(fro='io_Ndiv',to='lane_refclk_Ndiv')
        clockdivider.io_signals.mv(fro='io_shift',to='lane_refclk_shift')
        clockdivider.io_signals.mv(fro='io_clkpn',to='lane_refclk')

        #These are defined in scan input
        clockdivider.io_signals.mv(fro='io_reset_clk',to='lane_refclk_reset')
        clockdivider.io_signals.mv(fro='reset',to='reset_clock_div')

        # Add clocdivider and dut io signals to testbench connectors
        # Dut is creted automaticaly, if verilog file for it exists
        self.tb.connectors.update(bundle=clockdivider.io_signals.Members)
        self.tb.connectors.update(bundle=self.tb.dut_instance.io_signals.Members)
        for connector in self.scan.Data.Members['reset_sequence'].verilog_connectors:
            self.tb.connectors.Members[connector.name]=connector
            try: 
                self.dut.ios.Members[connector.name].connect=connector
            except:
                pass

            try: 
                clkdivider.ios.Members[connector.name].connect=connector
            except:
                pass

        # Some signals needed to control the sim
        self.tb.connectors.new(name='reset_loop', cls='reg') #Redundant?
        self.tb.connectors.new(name='asyncResetIn_clockRef', cls='reg') #Redundant?
        self.tb.connectors.new(name='lane_clkrst_asyncResetIn', cls='reg') #Redundant?


        ## Start initializations
        #Init the signals connected to the dut input to zero
        for name, val in self.tb.dut_instance.ios.Members.items():
            if val.cls=='input':
                val.connect.init='\'b0'

        #Init the signals connected to the clockdivider input
        for name, val in clockdivider.ios.Members.items():
            if val.cls=='input':
                val.connect.init='\'b0'

        # Connect helper diriving signals to their targets and deinit the targets.
        self.tb.connectors.connect(match=r"io_ctrl_and_clocks_.*_controls_.?_reset_loop",connect='reset_loop')
        self.tb.connectors.init(match=r"io_ctrl_and_clocks_.*_controls_.?_reset_loop",init='')
        self.tb.connectors.connect(match=r"io_ctrl_and_clocks_(dac|adc)_clocks_.?",connect='clock')
        self.tb.connectors.init(match=r"io_ctrl_and_clocks_(dac|adc)_clocks_.?",init='')
        self.tb.connectors.connect(match=r"io_lanes_tx_enq_clock",connect='lane_refclk')
        self.tb.connectors.init(match=r"io_lanes_tx_enq_clock",init='')

        # Init a selected set signals to chosen values
        # Some to ones
        oneslist=[
            'asyncResetIn_clockRef',
            'lane_clkrst_asyncResetIn',
            'io_ctrl_and_clocks_reset_index_count',
            'io_ctrl_and_clocks_adc_lut_reset',
            ]
        for serdes in range(self.nserdes):
            oneslist+=['io_lanes_tx_%s_ready' %(serdes), 'io_lanes_rx_%s_valid' %(serdes) ]

        for tx in range(self.Txantennas):
            for user in range(self.Users):
                oneslist+=['io_ctrl_and_clocks_tx_user_weights_%s_%s_real' %(tx,user),
                           'io_ctrl_and_clocks_tx_user_weights_%s_%s_imag' %(tx,user)]
        for rx in range(self.Rxantennas):
            for user in range(self.Users):
                oneslist+=['io_ctrl_and_clocks_rx_user_weights_%s_%s_real' %(rx,user),
                           'io_ctrl_and_clocks_rx_user_weights_%s_%s_imag' %(rx,user)]
        for scanind in range(self.nserdes+2):
            oneslist+=[ 'io_ctrl_and_clocks_from_serdes_scan_%s_valid' %(scanind),
            'io_ctrl_and_clocks_from_dsp_scan_%s_valid' %(scanind),
            'io_ctrl_and_clocks_to_serdes_mode_%s' %(scanind)]

        for scanind in range(self.nserdes+4):
            oneslist+=[ 'io_ctrl_and_clocks_to_dsp_mode_%s' %(scanind)]

        for name in oneslist:
            self.tb.connectors.Members[name].init='\'b1'

        # Initialize Selected signals with parameter values
        paramlist=[
                ('lane_refclk_Ndiv', 'g_lane_refclk_Ndiv'),
                ('lane_refclk_shift', 'g_lane_refclk_shift'),
                ('io_ctrl_and_clocks_tx_Ndiv', 'tx_c_ratio'),
                ('io_ctrl_and_clocks_tx_clkdiv_shift', 'g_tx_shift'),
                ('io_ctrl_and_clocks_rx_Ndiv', 'rx_c_ratio'),
                ('io_ctrl_and_clocks_rx_clkdiv_shift', 'g_rx_shift'),
                ('io_ctrl_and_clocks_user_spread_mode', 'g_tx_user_spread_mode'),
                ('io_ctrl_and_clocks_user_index', 'g_rx_user_index'),
                ('io_ctrl_and_clocks_antenna_index', 'g_rx_antenna_index'),
                ('io_ctrl_and_clocks_rx_output_mode', 'g_rx_output_mode'),
                ('io_ctrl_and_clocks_input_mode', 'g_rx_input_mode'),
                ('io_ctrl_and_clocks_adc_fifo_lut_mode', 'g_rx_adc_fifo_lut_mode')
            ]
        for tx in range(self.Txantennas):
            paramlist+=[
                ('io_ctrl_and_clocks_interpolator_controls_%s_cic3derivscale' %(tx), 
                    'g_tx_scale3'),
                ('io_ctrl_and_clocks_interpolator_controls_%s_cic3derivshift' %(tx), 
                    'g_tx_cic3shift'),
                ('io_ctrl_and_clocks_interpolator_controls_%s_hb1scale' %(tx), 
                    'g_tx_scale0'),
                ('io_ctrl_and_clocks_interpolator_controls_%s_hb2scale' %(tx), 
                    'g_tx_scale1'),
                ('io_ctrl_and_clocks_interpolator_controls_%s_hb3scale' %(tx), 
                    'g_tx_scale2'),
                ('io_ctrl_and_clocks_interpolator_controls_%s_mode' %(tx), 
                    'g_tx_interpolator_mode'),
                ('io_ctrl_and_clocks_user_sum_mode_%s' %(tx), 'g_tx_user_sum_mode'),
                ('io_ctrl_and_clocks_user_select_index_%s' %(tx), 'g_tx_user_select_index'),
                ('io_ctrl_and_clocks_dac_data_mode_%s' %(tx), 'g_tx_dac_data_mode')
            ]

        for rx in range(self.Rxantennas):
            paramlist+=[
                ('io_ctrl_and_clocks_decimator_controls_%s_cic3integscale' %(rx), 
                    'g_rx_scale0'),
                ('io_ctrl_and_clocks_decimator_controls_%s_cic3integshift' %(rx), 
                    'g_rx_cic3shift'),
                ('io_ctrl_and_clocks_decimator_controls_%s_hb1scale' %(rx), 
                    'g_rx_scale1'),
                ('io_ctrl_and_clocks_decimator_controls_%s_hb2scale' %(rx), 
                    'g_rx_scale2'),
                ('io_ctrl_and_clocks_decimator_controls_%s_hb3scale' %(rx), 
                    'g_rx_scale3'),
                ('io_ctrl_and_clocks_decimator_controls_%s_mode' %(rx), 
                    'g_rx_mode'),
                ('io_ctrl_and_clocks_inv_adc_clk_pol_%s' %(rx), 
                    'g_rx_inv_adc_clk_pol')
            ]
        # Loop to define the inits
        for name in paramlist:
            self.tb.connectors.Members[name[0]].init=name[1]

        # All signals are now available, it is possible to initialize 
        # Reset sequence
        # This does redefinitions to inits, it must be here
        #self.reset_sequence_gen()

        # IO file connector definitions
        # Define what signals and in which order and format are read form the files
        # i.e. verilog_connectors of the file
        name='Z'
        ionames=[]
        for count in range(self.Rxantennas):
            ionames+=[ 'io_Z_%s_real_t' %(count), 'io_Z_%s_real_b' %(count), 
            'io_Z_%s_imag_t' %(count), 'io_Z_%s_imag_b' %(count)]
        self.iofile_bundle.Members[name].verilog_connectors=\
                self.tb.connectors.list(names=ionames)
        #Format for thermobits is %b
        for val in self.iofile_bundle.Members[name].verilog_connectors:
            if re.match(r".*(real|imag)_t",val.name):
                val.ioformat="%b"

        name='io_lanes_tx'
        ionames=[]
        for user in range(self.Users):
            ionames+=['io_lanes_tx_0_bits_data_%s_udata_real' %(user),
                      'io_lanes_tx_0_bits_data_%s_udata_imag' %(user)]
        self.iofile_bundle.Members[name].verilog_connectors=\
                self.tb.connectors.list(names=ionames)
        # Change type to signed
        for name in ionames:
            self.tb.connectors.Members[name].type='signed'

        name='A'
        ionames=[]
        for count in range(self.Rxantennas):
            ionames+=['io_iptr_A_%s_real' %(count),
                     'io_iptr_A_%s_imag' %(count)]
        self.iofile_bundle.Members[name].verilog_connectors=\
                self.tb.connectors.list(names=ionames)
        
        name='io_lanes_rx'
        ionames=[]
        for serdes in range(self.nserdes):
            for user in range(self.Users):
                ionames+=['io_lanes_rx_%s_bits_data_%s_udata_real' %(serdes,user),
                          'io_lanes_rx_%s_bits_data_%s_udata_imag' %(serdes,user)]
        self.iofile_bundle.Members[name].verilog_connectors=\
                self.tb.connectors.list(names=ionames)

        name='serdestest_write'
        ionames=[]
        ionames+=['io_ctrl_and_clocks_serdestest_scan_write_mode',
                  'io_ctrl_and_clocks_serdestest_scan_write_address']
        #ionames.append('io_ctrl_and_clocks_serdestest_scan_write_en')
        #ionames.append('io_ctrl_and_clocks_serdestest_scan_write_value_rxindex')
        #for user in range(self.Users):
        #    ionames.append('io_ctrl_and_clocks_serdestest_scan_write_value_data_%s_udata_real' %(user))
        #    ionames.append('io_ctrl_and_clocks_serdestest_scan_write_value_data_%s_udata_imag' %(user))
        self.iofile_bundle.Members[name].verilog_connectors=\
                self.tb.connectors.list(names=ionames)

        # This should be a method too
        # Start the testbench contents
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


//Helper vars for simulation control
integer memaddrcount, initdone, rxdone, txdone;
initial memaddrcount=0;
initial initdone=0;
initial rxdone=0;
initial txdone=0;


//Clock divider model\n""" + clockdivider.instance + """//DUT definition
"""+self.tb.dut_instance.instance+"""

//Master clock is omnipresent
always #(c_Ts/2.0) clock = !clock;

//Execution with parallel fork-join and sequential begin-end sections
initial #0 begin
fork
""" + self.tb.connectors.verilog_inits(level=1)+""" 
//Tx_io
@(posedge initdone) begin
while (!txdone) begin
@(posedge io_ctrl_and_clocks_dac_clocks_0 ) begin
    //Print only valid values
    if ("""+self.iofile_bundle.Members['Z'].verilog_io_condition + """
    ) begin \n"""+ self.iofile_bundle.Members['Z'].verilog_io+"""
     end
end
end
end

//Rx_io
@(posedge initdone) begin
while (!rxdone) begin
@(posedge lane_refclk ) begin
//Mimic the reading to lanes
    //Print only valid values
    if ((io_lanes_tx_0_valid==1)  &&
    """+self.iofile_bundle.Members['io_lanes_tx'].verilog_io_condition + """
) begin
"""+self.iofile_bundle.Members['io_lanes_tx'].verilog_io + """
    end
end
end
end

"""+ self.reset_sequence()+"""
        // Sequence triggered by initdone
        @(posedge initdone) begin
            while (!$feof(f_io_lanes_rx)) begin
                 //Lane output fifo is read by the symrate clock
                 @(posedge io_lanes_rx_deq_clock )
                 """+ self.iofile_bundle.Members['io_lanes_rx'].verilog_io+"""
            end
            txdone<=1;
        end
        // Sequence triggered by initdone
        @(posedge initdone) begin
            while (!$feof(f_A)) begin
                @(posedge io_ctrl_and_clocks_adc_clocks_0 )
                """+self.iofile_bundle.Members['A'].verilog_io+"""
            end\n"""+self.iofile_bundle.Members['serdestest_write'].verilog_io +"""
            rxdone<=1;
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
    #print(t.tb.connector_definitions)
    #print(t.tb.definition)
    #t.tb.export(force=True)
    #print(t.tb.connector_definitions)

