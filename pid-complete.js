let clamp = function(val, low, high) {
  if (val > high) return high;
  if (val < low) return low;
  return val;
}


// Symbols for PID controller state
const DISABLED = Symbol();
const ENABLED_MANUAL = Symbol();
const ENABLED_AUTO = Symbol();


module.exports = function(RED) {
    function CompletePIDNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;

        node.name = config.name || "PID";
        node.sampleSec = parseFloat(config.sampleInterval) || 1;
        node.outMin = parseFloat(config.outMin) || 0;
        node.outMax = parseFloat(config.outMax) || 1;
        node.disabledOut = parseFloat(config.disabledOut) || 0;
        node.kp = null;
        node.ki = null;
        node.kd = null;
        node.sp = null;
        node.PonM = config.PonM || false;
        node.DonM = config.DonM || false;

        node.state = DISABLED;


        function reset(maintainIntegral = false) {
          node._proportional = 0;
          node._integral = maintainIntegral ? node._lastOutput : node.disabledOut;
          // node._integral = node.disabledOut;
          node._derivative = 0;

          node._lastTime = Date.now();
          node._lastInput = null;
          node._lastOutput = maintainIntegral ? node._lastOutput : node.disabledOut;
          node._lastError = null;
        }

        function setDisabled() {
          reset(false); //Do not need to save integral term if disabling
          node.state = DISABLED;
          node.status({fill:"grey", text:"Disabled"});
        }

        function setManual(output = null) {
          reset(false); //Do not need to save integral term, will be overwritten by manual output
          node._lastOutput = output || node._lastOutput;
          node._lastOutput = clamp(node._lastOutput, node.outMin, node.outMax);
          node._integral = node._lastOutput
          node.state = ENABLED_MANUAL;
          node.status({fill:"blue",text:"Manual"});
        }

        function setAuto() {
          if (node.state != ENABLED_AUTO) {

            let maintainIntegral = true; //Default to maintaining integral from manual operation to maintain stable PV
            if (node.state === DISABLED) maintainIntegral = false; //If we were disabled, there is no reason to save integral term

            reset(maintainIntegral);

            if (node.kp != null && node.ki != null && node.kd != null && node.sp != null) {
              node._integral = clamp(node._integral, node.outMin, node.outMax);
              node.state = ENABLED_AUTO;
              node.status({fill:"green", text:"Auto"});
            } else {
              node.status({fill:"red", text:"Setpoint or Tuning not initialized."});
            }
          }
        }

        function calculate(input) {
          let now = Date.now();
          let dt = (now - (node._lastTime || 1e-16)) / 1000;
          if (dt < node.sampleSec) return node._lastOutput;

          let error = node.sp - input;
          let dInput = input - (node._lastInput || input);
          let dError = error - (node._lastError || error);

          if (node.PonM) {
            node._proportional -= node.kp * dInput;
          } else {
            node._proportional = node.kp * error;
          }

          node._integral += node.ki * error * dt;
          node._integral = clamp(node._integral, node.outMin, node.outMax);

          if (node.DonM) {
            node._derivative = -1 * node.kd * dInput;
          } else {
            node._derivative = node.kd * dError;
          }

          let output = node._proportional + node._integral + node._derivative;
          output = clamp(output, node.outMin, node.outMax);

          node._lastTime = now;
          node._lastOutput = output;
          node._lasInput = input;
          node._lastError = error;
        }

        reset(false);

        node.on('input', function(msg, send, done) {
          if (msg.hasOwnProperty("cmd")) {
            switch (msg.cmd) {
              case "reset":
                reset();
                break;
              case "disable":
                setDisabled();
                break;
              case "manual":
                let output = null;
                if (msg.hasOwnProperty("payload")) {
                  output = parseFloat(msg.payload);
                }
                setManual(output);
                break;
              case "auto":
                setAuto();
                break;
            }
          } else if (msg.hasOwnProperty("topic")) {
            if (msg.hasOwnProperty("payload")) {
              switch (msg.topic) {
                case "setpoint":
                case "sp":
                  node.sp = parseFloat(msg.payload);
                  break;
                case "kp":
                  node.kp = parseFloat(msg.payload);
                  break;
                case "ki":
                  node.ki = parseFloat(msg.payload);
                  break;
                case "kd":
                  node.kd = parseFloat(msg.payload);
                  break;
              }
            }

          } else if (msg.hasOwnProperty("payload")) {
            if (node.state === ENABLED_AUTO) {
              let tempPV = parseFloat(msg.payload);
              if (!isNaN(tempPV)) {
                calculate(tempPV);
              }
            }
          }

          let newMsg = {
                        name: node.name,
                        payload: node._lastOutput
                      };

          switch (node.state) {
            case DISABLED:
              newMsg.state = "Disabled";
              break;
            case ENABLED_MANUAL:
              newMsg.state = "Manual";
              break;
            case ENABLED_AUTO:
              newMsg.state = "Auto";
              newMsg.proportional = node._proportional;
              newMsg.integral = node._integral;
              newMsg.derivative = node._derivative;
              break;
          }

          send(newMsg);

          if (done) done();
        });
    }
    RED.nodes.registerType("pid-complete",CompletePIDNode);
}
