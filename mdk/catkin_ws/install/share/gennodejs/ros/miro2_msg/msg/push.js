// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class push {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link = null;
      this.flags = null;
      this.pushpos = null;
      this.pushvec = null;
    }
    else {
      if (initObj.hasOwnProperty('link')) {
        this.link = initObj.link
      }
      else {
        this.link = 0;
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('pushpos')) {
        this.pushpos = initObj.pushpos
      }
      else {
        this.pushpos = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('pushvec')) {
        this.pushvec = initObj.pushvec
      }
      else {
        this.pushvec = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type push
    // Serialize message field [link]
    bufferOffset = _serializer.int32(obj.link, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [pushpos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.pushpos, buffer, bufferOffset);
    // Serialize message field [pushvec]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.pushvec, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type push
    let len;
    let data = new push(null);
    // Deserialize message field [link]
    data.link = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pushpos]
    data.pushpos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [pushvec]
    data.pushvec = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/push';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6271e5b7c8f54208b938d70e5eaafbe6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #	@section COPYRIGHT
    #	Copyright (C) 2019 Consequential Robotics Ltd
    #	
    #	@section AUTHOR
    #	Consequential Robotics http://consequentialrobotics.com
    #	
    #	@section LICENSE
    #	For a full copy of the license agreement, and a complete
    #	definition of "The Software", see LICENSE in the MDK root
    #	directory.
    #	
    #	Subject to the terms of this Agreement, Consequential
    #	Robotics grants to you a limited, non-exclusive, non-
    #	transferable license, without right to sub-license, to use
    #	"The Software" in accordance with this Agreement and any
    #	other written agreement with Consequential Robotics.
    #	Consequential Robotics does not transfer the title of "The
    #	Software" to you; the license granted to you is not a sale.
    #	This agreement is a binding legal agreement between
    #	Consequential Robotics and the purchasers or users of "The
    #	Software".
    #	
    #	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
    #	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    #	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    #	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
    #	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
    #	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    #	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    #	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    #	
    #
    #	Description... TODO.
    
    #	The identifier of the pushed link
    int32 link
    
    #	DOCLINK PUSH FLAGS
    #
    #	The values of these flags are defined in miro2.constants
    #
    #	PUSH_FLAG_IMPULSE
    #		Treat push as an impulse (in mm).
    #
    #	PUSH_FLAG_VELOCITY
    #		Treat push as a velocity (in mm/sec).
    #
    #	NB: If neither of the above flags is set, the push should
    #	not be actioned at all (this condition is used in server
    #	to indicate "not pending").
    #
    #	PUSH_FLAG_NO_TRANSLATION
    #		Zero out any resultant change in pose.dr (i.e. only
    #		turn on the spot).
    #
    #	PUSH_FLAG_NO_ROTATION
    #		Zero out any resultant change in pose.dtheta /and/
    #		in pose.dr (i.e. do not move wheels at all).
    #
    #	PUSH_FLAG_NO_NECK_MOVEMENT
    #		Zero out any resultant change in neck configuration;
    #		This flag is independent of NO_TRANSLATION/ROTATION.
    #
    #	PUSH_FLAG_WAIT
    #		Cause the push processor to wait for further pushes
    #		before publishing a velocity at its output. This is
    #		required if you want to pass multiple push streams.
    uint32 flags
    
    #	The pushed point, in the reference frame of the pushed link
    geometry_msgs/Point pushpos
    
    #	The push vector, in the reference frame of the pushed link
    geometry_msgs/Vector3 pushvec
    
    
    
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new push(null);
    if (msg.link !== undefined) {
      resolved.link = msg.link;
    }
    else {
      resolved.link = 0
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.pushpos !== undefined) {
      resolved.pushpos = geometry_msgs.msg.Point.Resolve(msg.pushpos)
    }
    else {
      resolved.pushpos = new geometry_msgs.msg.Point()
    }

    if (msg.pushvec !== undefined) {
      resolved.pushvec = geometry_msgs.msg.Vector3.Resolve(msg.pushvec)
    }
    else {
      resolved.pushvec = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = push;
