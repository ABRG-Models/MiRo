// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let affect_adjust = require('./affect_adjust.js');
let sleep_adjust = require('./sleep_adjust.js');

//-----------------------------------------------------------

class animal_adjust {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mood = null;
      this.emotion = null;
      this.sleep = null;
    }
    else {
      if (initObj.hasOwnProperty('mood')) {
        this.mood = initObj.mood
      }
      else {
        this.mood = new affect_adjust();
      }
      if (initObj.hasOwnProperty('emotion')) {
        this.emotion = initObj.emotion
      }
      else {
        this.emotion = new affect_adjust();
      }
      if (initObj.hasOwnProperty('sleep')) {
        this.sleep = initObj.sleep
      }
      else {
        this.sleep = new sleep_adjust();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type animal_adjust
    // Serialize message field [mood]
    bufferOffset = affect_adjust.serialize(obj.mood, buffer, bufferOffset);
    // Serialize message field [emotion]
    bufferOffset = affect_adjust.serialize(obj.emotion, buffer, bufferOffset);
    // Serialize message field [sleep]
    bufferOffset = sleep_adjust.serialize(obj.sleep, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type animal_adjust
    let len;
    let data = new animal_adjust(null);
    // Deserialize message field [mood]
    data.mood = affect_adjust.deserialize(buffer, bufferOffset);
    // Deserialize message field [emotion]
    data.emotion = affect_adjust.deserialize(buffer, bufferOffset);
    // Deserialize message field [sleep]
    data.sleep = sleep_adjust.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/animal_adjust';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b26581aa1bf2879431400970feb511a2';
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
    #	Animal adjust provides a route for directly adjusting the
    #	animal state. See message "animal_state" for a description
    #	of the state itself; see message "adjust" for details of
    #	how adjustment can be performed.
    
    affect_adjust mood
    affect_adjust emotion
    sleep_adjust sleep
    
    
    ================================================================================
    MSG: miro2_msg/affect_adjust
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
    
    adjust valence
    adjust arousal
    
    
    ================================================================================
    MSG: miro2_msg/adjust
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
    
    
    ================================================================================
    MSG: miro2_msg/sleep_adjust
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
    
    adjust wakefulness
    adjust pressure
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new animal_adjust(null);
    if (msg.mood !== undefined) {
      resolved.mood = affect_adjust.Resolve(msg.mood)
    }
    else {
      resolved.mood = new affect_adjust()
    }

    if (msg.emotion !== undefined) {
      resolved.emotion = affect_adjust.Resolve(msg.emotion)
    }
    else {
      resolved.emotion = new affect_adjust()
    }

    if (msg.sleep !== undefined) {
      resolved.sleep = sleep_adjust.Resolve(msg.sleep)
    }
    else {
      resolved.sleep = new sleep_adjust()
    }

    return resolved;
    }
};

module.exports = animal_adjust;
