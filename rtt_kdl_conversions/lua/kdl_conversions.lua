local rttlib = require "rttlib"
local rtt = rtt
module("kdl_conversions")

function frame_to_msg(f)
   msg = rtt.Variable("geometry_msgs/Pose")
   rtt.provides("KDL"):FrameToMsg(f,msg)
   return msg
end
function msg_to_frame(msg)
   f = rtt.Variable("KDL/Frame")
   rtt.provides("KDL"):MsgToFrame(msg,f)
   return f
end

