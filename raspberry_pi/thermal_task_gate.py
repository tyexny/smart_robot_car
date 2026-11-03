"""Read-only, identity-correlated FPGA ARRIVED confirmation before acquisition."""
import secrets
import socket
import struct
import time

REQUEST=struct.Struct('!4sBBBBIHHHH')
HEADER=struct.Struct('!4sBBHIHH')

# A target that stays locked this long opens the camera without arrival.
LOCK_SECONDS=3.0

class ThermalTaskGate:
    def __init__(self, provider, destination, *, clock=time.monotonic):
        self.provider,self.destination,self.clock=provider,destination,clock
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('',0));self.sock.settimeout(.03)
        self.sequence=secrets.randbits(32)

    def __call__(self):
        controller=self.provider()
        goal=getattr(controller,'goal',None)
        if not goal or not getattr(goal,'thermal_check',False) or controller.phase not in ('approach','arrival_pending','stopped','thermal'):
            return False
        identity=(getattr(controller,'session',0),getattr(controller,'selected_track',getattr(controller,'track_id',0)))
        if not all(identity):return False
        # Option A: thermal-check tasks open the camera once the target has
        # held its lock for LOCK_SECONDS, long before the arrival query answers.
        locked_at=getattr(controller,'lock_confirmed_at',None)
        if locked_at is not None and self.clock()-locked_at>=LOCK_SECONDS:
            return True
        self.sequence=(self.sequence+1)&0xffffffff
        packet=REQUEST.pack(b'FTIQ',1,3,0,0,self.sequence,0,0,self.sock.getsockname()[1],0)
        try:
            self.sock.sendto(packet,self.destination)
            deadline=self.clock()+.12
            while self.clock()<deadline:
                self.sock.settimeout(min(.03,max(.001,deadline-self.clock())))
                try:
                    raw,peer=self.sock.recvfrom(128)
                except socket.timeout:
                    if self.clock()<deadline:self.sock.sendto(packet,self.destination)
                    continue
                if peer[0]!=self.destination[0] or len(raw)!=26:continue
                magic,version,status,count,sequence,register,reserved=HEADER.unpack_from(raw)
                if (magic,version,status,count,sequence,register,reserved)!=(b'FTRP',1,0,5,self.sequence,0,0):continue
                session,track,flags=struct.unpack_from('!IIH',raw,16)
                # Recheck after network I/O: a new command cannot inherit an old result.
                if self.provider() is not controller or getattr(controller,'goal',None) is not goal:return False
                if controller.phase not in ('approach','arrival_pending','stopped','thermal'):return False
                current=(getattr(controller,'session',0),getattr(controller,'selected_track',getattr(controller,'track_id',0)))
                if current!=identity:return False
                if flags==1 and (session,track)==identity:
                    controller.phase='thermal';controller.last_error='已到达目标，正在检测温度'
                    return True
                break
        except OSError:
            pass
        if (self.provider() is controller and getattr(controller,'goal',None) is goal and
                controller.phase=='arrival_pending' and
                self.clock()-getattr(controller,'arrival_pending_at',self.clock())>2):
            controller.abort('未收到对应目标的到达确认，未开启温检')
        return False

    def close(self):
        self.sock.close()
