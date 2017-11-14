# f2_dsp class 
# Last modification by Marko Kosunen, marko.kosunen@aalto.fi, 13.11.2017 19:12
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
            out=resampled.T
            
            #Matched filter for sync
            Hstf=self.Hstf
            Hltf=self.Hltf
            print(len(Hstf))

            print(len(Hltf))
            framesyncshort=np.abs(sig.convolve(resampled.T, Hstf, mode='full'))**2
            framesynclong=np.abs(len(Hstf)/len(Hltf)*sig.convolve(resampled.T, Hltf, mode='full'))**2
            Efil=np.ones((6,1))
            Sstf=sig.convolve(framesyncshort,Efil,mode='full')
            Sltf=sig.convolve(framesynclong,Efil,mode='full')
            Sfil=np.zeros((65,1))
            Sfil[16:65:16]=1
            Snstf=sig.convolve(Sstf,Sfil,mode='full')/4
            a=np.r_['0',Sltf, np.zeros((len(Snstf)-len(Sltf),1))]
            Sn=a+Snstf
            if par:
                queue.put(out)
                queue.put(Snstf)
                queue.put(Sn)
            #print(out.shape)
            #print(framesync.shape)
            self._Z.Value=out
            self._Frame_sync_short.Value=Snstf
            self._Frame_sync_long.Value=Sn

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

