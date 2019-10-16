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

class animal_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flags = null;
      this.emotion = null;
      this.mood = null;
      this.sleep = null;
      this.time_of_day = null;
      this.sound_level = null;
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
      if (initObj.hasOwnProperty('time_of_day')) {
        this.time_of_day = initObj.time_of_day
      }
      else {
        this.time_of_day = 0.0;
      }
      if (initObj.hasOwnProperty('sound_level')) {
        this.sound_level = initObj.sound_level
      }
      else {
        this.sound_level = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type animal_state
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [emotion]
    bufferOffset = affect.serialize(obj.emotion, buffer, bufferOffset);
    // Serialize message field [mood]
    bufferOffset = affect.serialize(obj.mood, buffer, bufferOffset);
    // Serialize message field [sleep]
    bufferOffset = sleep.serialize(obj.sleep, buffer, bufferOffset);
    // Serialize message field [time_of_day]
    bufferOffset = _serializer.float32(obj.time_of_day, buffer, bufferOffset);
    // Serialize message field [sound_level]
    bufferOffset = _serializer.float32(obj.sound_level, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type animal_state
    let len;
    let data = new animal_state(null);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [emotion]
    data.emotion = affect.deserialize(buffer, bufferOffset);
    // Deserialize message field [mood]
    data.mood = affect.deserialize(buffer, bufferOffset);
    // Deserialize message field [sleep]
    data.sleep = sleep.deserialize(buffer, bufferOffset);
    // Deserialize message field [time_of_day]
    data.time_of_day = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sound_level]
    data.sound_level = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/animal_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02b89a84b06f59e91819662e7c3d6b0e';
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
    # the "animal_state" is the state of the animal aspects of the model
    # which includes "affect" values for emotion (quickly changing) and
    # mood (slowly changing) and a "sleep" value (also slowly changing),
    # and the animal's estimate of time. this state may also include, in
    # future, physical states such as temperature.
    
    #	DOCLINK ANIMAL STATE FLAGS
    #
    #	Some flags are included here because parts of the implementation
    #	are in separate nodes that read this topic in order to determine
    #	how they should behave, and their behaviour is affected by flags.
    #
    #	The values of these flags are defined in miro2.constants.
    uint32 flags
    
    # affective states
    affect emotion
    affect mood
    
    # sleep state
    sleep sleep
    
    # normalised time of day (0.0 -> 1.0)
    float32 time_of_day
    
    # normalised ambient sound level (0.0 -> 1.0)
    float32 sound_level
    
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
    const resolved = new animal_state(null);
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

    if (msg.time_of_day !== undefined) {
      resolved.time_of_day = msg.time_of_day;
    }
    else {
      resolved.time_of_day = 0.0
    }

    if (msg.sound_level !== undefined) {
      resolved.sound_level = msg.sound_level;
    }
    else {
      resolved.sound_level = 0.0
    }

    return resolved;
    }
};

module.exports = animal_state;
