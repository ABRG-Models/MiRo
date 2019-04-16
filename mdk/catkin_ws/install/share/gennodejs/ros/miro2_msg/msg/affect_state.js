// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let affect = require('./affect.js');
let sleep = require('./sleep.js');

//-----------------------------------------------------------

class affect_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flags = null;
      this.emotion = null;
      this.mood = null;
      this.sleep = null;
    }
    else {
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('emotion')) {
        this.emotion = initObj.emotion
      }
      else {
        this.emotion = new affect();
      }
      if (initObj.hasOwnProperty('mood')) {
        this.mood = initObj.mood
      }
      else {
        this.mood = new affect();
      }
      if (initObj.hasOwnProperty('sleep')) {
        this.sleep = initObj.sleep
      }
      else {
        this.sleep = new sleep();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type affect_state
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [emotion]
    bufferOffset = affect.serialize(obj.emotion, buffer, bufferOffset);
    // Serialize message field [mood]
    bufferOffset = affect.serialize(obj.mood, buffer, bufferOffset);
    // Serialize message field [sleep]
    bufferOffset = sleep.serialize(obj.sleep, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type affect_state
    let len;
    let data = new affect_state(null);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [emotion]
    data.emotion = affect.deserialize(buffer, bufferOffset);
    // Deserialize message field [mood]
    data.mood = affect.deserialize(buffer, bufferOffset);
    // Deserialize message field [sleep]
    data.sleep = sleep.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/affect_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8eba44c465d72613e49b25d7a944efcf';
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
    
    #	DOCLINK AFFECT FLAGS
    #
    #	The values of these flags are defined in miro2.constants
    #
    #	AFFECT_EXPRESS_THROUGH_VOICE
    #		Enable vocal expression of affective state
    #
    #	AFFECT_EXPRESS_THROUGH_BODY
    #		Enable body expression of affective state
    uint32 flags
    
    # the "affect_state" is the complete affective state, including
    # "affect" values for emotion (quickly changing) and mood (slowly
    # changing) and a "sleep" value (also slowly changing).
    affect emotion
    affect mood
    sleep sleep
    
    
    ================================================================================
    MSG: miro2_msg/affect
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
    #	The "affect" state is two-dimensional, encoding valence
    #	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).
    #	The states are usually driven by signals entering the robot's
    #	sensory systems, but can also be driven directly by other systems.
    
    float32 valence
    float32 arousal
    
    
    ================================================================================
    MSG: miro2_msg/sleep
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
    #	The "sleep" state is two-dimensional, encoding "wakefulness"
    #	(0.0 to 1.0, what it sounds like) and "pressure" (0.0 to 1.0,
    #	tendency to move towards reduced wakefulness). The two states
    #	evolve together to implement a relaxation oscillator.
    
    float32 wakefulness
    float32 pressure
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new affect_state(null);
    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.emotion !== undefined) {
      resolved.emotion = affect.Resolve(msg.emotion)
    }
    else {
      resolved.emotion = new affect()
    }

    if (msg.mood !== undefined) {
      resolved.mood = affect.Resolve(msg.mood)
    }
    else {
      resolved.mood = new affect()
    }

    if (msg.sleep !== undefined) {
      resolved.sleep = sleep.Resolve(msg.sleep)
    }
    else {
      resolved.sleep = new sleep()
    }

    return resolved;
    }
};

module.exports = affect_state;
