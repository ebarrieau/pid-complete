let clamp = function(val, low, high) {
  if (val > high) return high;
  if (val < low) return low;
  return val;
}


// Symbols for PID controller state
const DISABLED = Symbol();
const ENABLED_MANUAL = Symbol();
const ENABLED_AUTO = Symbol();
const ERROR = Symbol();


module.exports = function(RED) {
    function CompletePIDNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;

        node.name = config.name || "PID";
        node.sampleSec = parseFloat(config.sampleInterval);
        node.outMin = parseFloat(config.outMin);
        node.outMax = parseFloat(config.outMax);
        node.disabledOut = parseFloat(config.disabledOut);
        node.PonM = config.PonM;
        node.DonM = config.DonM;

        node.kpName = config.kp;
        node.kpType = config.kpType;
        node.kiName = config.ki;
        node.kiType = config.kiType;
        node.kdName = config.kd;
        node.kdType = config.kdType;

        node.kp = null;
        node.ki = null;
        node.kd = null;
        node.sp = null;

        let configError = [node.sampleSec,
                           node.outMin,
                           node.outMax,
                           node.disabledOut].some( (element) => {
                             return isNaN(element);
                           });

        node.invert = 1; //Default to not inverting the controller action
        if (node.outMax < node.outMin) {
          // Controller action is reverse (e.g. cooling or braking) so swap
          // the output ranges and set the invert flag to -1
          node.invert = -1;
          let tempMin = node.outMax;
          node.outMax = node.outMin;
          node.outMin = tempMin;
        }

        node.state = configError ? ERROR : DISABLED;

        node.get = function (prop, type, gain) {
          let output = undefined;
          switch (type) {
            case "global":
              output = node.context().global.get(prop);
              break;
            case "flow":
              output = node.context().flow.get(prop);
              break;
          }
          if (output != undefined && isFinite(output) && !isNaN(output)) {
            node[gain] = output;
          }
        }

        node.updateKp = function() {
          node.get(node.kpName, node.kpType, "kp");
        }

        node.updateKi = function() {
          node.get(node.kiName, node.kiType, "ki");
        }

        node.updateKd = function() {
          node.get(node.kdName, node.kdType, "kd");
        }

        node.set = function (prop, type, gain, val) {
          if (val != undefined && isFinite(val) && !isNaN(val)) {
            node[gain] = val;
            switch (type) {
              case "global":
                node.context().global.set(prop, val);
                break;
              case "flow":
                output = node.context().flow.set(prop, val);
                break;
            }
          }
        }

        // Helper functions to save the current values to both the context
        // store if one is set and to the internal variable

        node.setSp = function(val) {
          node.set(null, null, "sp", val);
        }

        node.setKp = function(val) {
          node.set(node.kpName, node.kpType, "kp", val)
        }

        node.setKi = function(val) {
          node.set(node.kiName, node.kiType, "ki", val)
        }

        node.setKd = function(val) {
          node.set(node.kdName, node.kdType, "kd", val)
        }

        //Call the update functions once to initialize the tuning variables if
        //context storage is used and there is valid data in context
        let initializeGain = function(prop, type, getFn, setFn) {
          if (type === "num") {
            setFn(prop);
          } else {
            getFn();
          }
        }

        initializeGain(node.kpName, node.kpType, node.updateKp, node.setKp);
        initializeGain(node.kiName, node.kiType, node.updateKi, node.setKi);
        initializeGain(node.kdName, node.kdType, node.updateKd, node.setKd);


        function reset(maintainIntegral = false) {
          node._proportional = 0;
          node._integral = maintainIntegral ? node._integral : node.disabledOut;
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
          let tempLastOutput = node._lastOutput;
          reset(false); //Do not need to save integral term, will be overwritten by manual output
          if (output != null && !isNaN(output)) {
            node._lastOutput = output;
          } else {
            node._lastOutput = tempLastOutput;
          }

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

          let error = (node.sp - input) * node.invert;
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
                  node.setSp(msg.payload);
                  break;
                case "kp":
                  node.setKp(msg.payload);
                  break;
                case "ki":
                  node.setKi(msg.payload);
                  break;
                case "kd":
                  node.setKd(msg.payload);
                  break;
              }
            }

          } else if (msg.hasOwnProperty("payload")) {
            if (node.state === ENABLED_AUTO) {
              let tempPV = parseFloat(msg.payload);
              if (tempPV != undefined && isFinite(tempPV) && !isNaN(tempPV)) {
                node.updateKp();
                node.updateKi();
                node.updateKd();
                calculate(tempPV);
              }
            }
          }

          let newMsg = {
                        name: node.name,
                        //this line ensures that no output value will fall outside the max and min allowable range
                        payload: clamp(node._lastOutput, node.outMin, node.outMax)
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
            case ERROR:
              newMsg.state = "Config Error";
          }

          send(newMsg);

          if (done) done();
        });
    }
    RED.nodes.registerType("pid-complete",CompletePIDNode);
}
