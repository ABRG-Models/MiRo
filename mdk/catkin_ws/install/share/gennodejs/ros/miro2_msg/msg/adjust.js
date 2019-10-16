// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class adjust {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
      this.gamma = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0.0;
      }
      if (initObj.hasOwnProperty('gamma')) {
        this.gamma = initObj.gamma
      }
      else {
        this.gamma = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type adjust
    // Serialize message field [data]
    bufferOffset = _serializer.float32(obj.data, buffer, bufferOffset);
    // Serialize message field [gamma]
    bufferOffset = _serializer.float32(obj.gamma, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type adjust
    let len;
    let data = new adjust(null);
    // Deserialize message field [data]
    data.data = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gamma]
    data.gamma = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/adjust';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f57be85f314b6a6c4ab78af0ca827a06';
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
    #	Adjust message provides a route for directly adjusting
    #	a state of the biomimetic model. There are two ways to
    #	specify an adjustment, selected independently for each
    #	adjustment channel.
    #
    #	1) Provide a target value in "data" and a "gamma" value
    #	between 0 and 1 to cause the state to approach the target:
    #
    #	(at 50Hz)
    #	state += gamma * (data - state)
    #
    #	2) Provide a delta value in "data" and set "gamma"
    #	to -1 to indicate this drive mode:
    #
    #	(at 50Hz)
    #	state += data
    #
    #	Understood values of gamma, therefore, are:
    #	   -1 : add "data" to state
    #	    0 : do nothing
    #	  0-1 : move state towards "data"
    #	    1 : instantly set state to "data"
    
    float32 data
    float32 gamma
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new adjust(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0.0
    }

    if (msg.gamma !== undefined) {
      resolved.gamma = msg.gamma;
    }
    else {
      resolved.gamma = 0.0
    }

    return resolved;
    }
};

module.exports = adjust;
