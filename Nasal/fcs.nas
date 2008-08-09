var FCSFilter = {
  new : func(input_path, output_path) {
    var obj = { parents : [FCSFilter], 
                input_path : input_path,
                output_path : output_path };
    obj.axis_conv = {'roll' : 'aileron', 'pitch' : 'elevator', 'yaw' : 'rudder' };
    return obj;
  },

  # read input command for a given axis
  read : func(axis) {
    if (me.input_path == nil or me.input_path == "") {
      return getprop("/controls/flight/" ~ me.axis_conv[axis]);
    } else { 
      return getprop(me.input_path ~ "/" ~ axis);
    }
  },

  # write output command for a given axis
  # this will be the output of an next command filter (like SAS)
  write : func(axis, value) {
    if (me.output_path == nil or me.output_path == '') {
      setprop("/controls/flight/fcs/" ~ axis, value);
    } else {
      setprop(me.output_path ~ "/" ~ axis, value);
    }
  }
};


var SAS = {
  new : func(initial_gains, sensitivities, authority_limit, input_path, output_path) {
    var obj = FCSFilter.new(input_path, output_path);
    obj.parents = [FCSFilter, SAS];
    obj.authority_limit = authority_limit;
    obj.sensitivities = sensitivities; 
    obj.initial_gains = initial_gains;
    return obj;
  },

  calcGain : func(axis) {
    var mach = getprop("/velocities/mach");
    var initial_gain = me.initial_gains[axis];
    var gain = initial_gain - 0.1 * mach * mach;
    if (math.abs(gain) < math.abs(initial_gain) * 0.01 or gain * initial_gain < 0) {
      gain = initial_gain * 0.01;
    }
    return gain;
  }, 

  apply : func(axis) {
    var input = me.read(axis);
    var mach = getprop("/velocities/mach");
    var value = 0;
    var rate = getprop("/orientation/" ~ axis ~ "-rate-degps");
    var gain = me.calcGain(axis);
    if (rate >= me.sensitivities[axis]) {
      value = - gain * rate;
      if (value > me.authority_limit) {
        value = me.authority_limit;
      } else if (value < - me.authority_limit) {
        value = - me.authority_limit;
      } 
    }
    me.write(axis, value + input);
  }
};


var CAS = {
  new : func(initial_gains, sensitivities, input_path, output_path) {
    var obj = FCSFilter.new(input_path, output_path);
    obj.parents = [FCSFilter, CAS];
    obj.sensitivities = sensitivities; 
    obj.initial_gains = initial_gains;
    return obj;
  },

  # FIXME: command for CAS is just a temporal one
  calcCommand: func (axis, input) {
    var gain = me.calcGain(axis);
    var target_rate = input * gain;
    var rate = getprop("/orientation/" ~ axis ~ "-rate-degps");
    var drate = target_rate - rate;
    var target_input = drate / gain;
    return target_input;
  },

  # FixMe: gain should be calculated using both speed and dynamic pressure
  calcGain : func(axis) {
    var mach = getprop("/velocities/mach");
    var gain = me.initial_gains[axis] - 0.1 * mach * mach;
    if (gain * me.initial_gains[axis] < 0.0 ) {
      gain = 0;
    }
    return gain;
  }, 

  apply : func(axis) {
    var input = me.read(axis);
    var cas_command = me.calcCommand(axis, input);
    me.write(axis, cas_command);
  }
};


var sas = nil;
var cas = nil;
var sensitivities = {'roll' : 1, 'pitch' : 0.5, 'yaw' : 0.5 };
var sas_initial_gains = {'roll' : 0.02, 'pitch' : -0.08, 'yaw' : 0.03 };
var cas_initial_gains = {'roll' : 30, 'pitch' : -10, 'yaw' : 15 };

var update = func {
  if (cas != nil) {
    cas.apply('roll');
    cas.apply('pitch');
    cas.apply('yaw');
  }
  sas.apply('roll');
  sas.apply('pitch');
  sas.apply('yaw');
  settimer(update, 0);
}

var initialize = func {
#  cas = CAS.new(cas_initial_gains, sensitivities, nil, "/controls/flight/fcs/cas");
#  sas = SAS.new(sas_initial_gains, sensitivities, 0.4, "/controls/flight/fcs/cas", "/controls/flight/fcs");
  sas = SAS.new(sas_initial_gains, sensitivities, 0.35, nil, "/controls/flight/fcs");
  settimer(update, 0);
}

_setlistener("/sim/signals/fdm-initialized", initialize);
