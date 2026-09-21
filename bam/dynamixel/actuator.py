# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
from bam.message import yellow, print_parameter, bright
from bam.actuator import VoltageControlledActuator, CurrentControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench, Pendulum


class MXActuator(VoltageControlledActuator):
    """
    Represents a Dynamixel MX-64 or MX-106 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=15.0,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and MX actuators
            error_gain=0.158,
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm=0.9625,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 5.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


MX28_ENCODER_COUNTS_PER_REV = 4096  # Resolution: 4096 pulse/rev (datasheet)
MX28_KP_DIVISOR = 128  # Position P Gain: KPP = KPP(TBL) / 128 (datasheet, not empirically corrected)
MX28_PWM_LIMIT = 885   # Default Present PWM limit for MX-28(2.0) (datasheet)


class MX28Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel MX-28(2.0) actuator.

    Protocol 2.0 control table -- identical layout to XH540 (Torque
    Enable=64, Position P/I/D Gain=84/82/80 with the same KPP=register/128
    conversion, Goal Position=116, Present PWM=124, PWM Limit=36, all 4096
    counts/rev), unlike the original Protocol 1.0 MX-28/64/106. Not
    inherited from MXActuator: every constructor argument differs (formula-
    based error_gain instead of an oscilloscope constant), so there's
    nothing to actually share.

    See bam/dynamixel/configure_kp_divisor.py to (re-)measure
    MX28_KP_DIVISOR directly from the servo -- the same script and
    DynamixelXH540 class work unmodified for MX-28(2.0), just point
    --port/--id at it.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=16.5,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            error_gain=(MX28_ENCODER_COUNTS_PER_REV / (2 * np.pi))
            / (MX28_KP_DIVISOR * MX28_PWM_LIMIT),
            max_pwm=1.0,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        # Center from datasheet stall torque/current @12V (2.5/1.4 ~= 1.79 Nm/A),
        # range widened to also cover the no-load-speed cross-check
        # (12V / 55rpm ~= 2.08 Nm/A)
        self.model.kt = Parameter(1.9, 1.2, 2.5)

        # Motor resistance [Ohm]
        # Stall condition U ~= I*R across 11.1/12.0/14.8V all agree on ~8.5-8.7 Ohm.
        # Upper bound temporarily widened 11.0 -> 15.0 to diagnose the fit pinning
        # R against the old bound (likely compensating for MX28_KP_DIVISOR=128
        # being uncorrected -- see the constant above); revert once that's
        # resolved and R settles away from the boundary again.
        self.model.R = Parameter(8.6, 6.0, 15.0)

        # Motor armature / apparent inertia [kg m^2]
        # No rotor inertia published, left for the optimizer to identify.
        # Lower bound 0.0005 -> 0.0: physically armature can't be negative,
        # so 0.0 is a valid floor (was previously kept slightly off zero for
        # no particular reason tied to this actuator).
        self.model.armature = Parameter(0.003, 0.0, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


MX106V2_ENCODER_COUNTS_PER_REV = 4096  # Resolution: 4096 pulse/rev (datasheet)
MX106V2_KP_DIVISOR = 147  # Empirically measured with bam/dynamixel/configure_kp_divisor.py
# (n-weighted mean across kp in [100, 200, 300, 400, 600, 800], individual
# per-kp estimates in [144.7, 149.0]; datasheet/manual says 128 -- same kind
# of mismatch previously found for XL330, see XL330_KP_DIVISOR below.
# TODO: re-run and use the script's own pooled least-squares estimate
# (printed at the end of a run) instead of this approximation.
MX106V2_PWM_LIMIT = 885   # Present PWM limit, read back from the servo's own register (confirmed)


class MX106V2Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel MX-106(2.0) actuator, i.e. the Protocol 2.0
    firmware variant -- NOT the same as the "mx106" key elsewhere in this
    repo, which targets the original Protocol 1.0 MX-106 via MXActuator
    (oscilloscope-measured error_gain=0.158) and has its own fitted params
    under bam/params/mx106/. Control table layout is identical to XH540
    (Torque Enable=64, Position P/I/D Gain=84/82/80 with the same
    KPP=register/MX106V2_KP_DIVISOR conversion, Goal Position=116, Present
    PWM=124, PWM Limit=36), so bam/dynamixel/configure_kp_divisor.py and
    DynamixelXH540 work unmodified for it -- just point --port/--id at it.

    kt/R/armature below are carried over unchanged from MXActuator
    (physical motor is the same across protocol versions); only error_gain
    was re-derived for this firmware's control table / KP scaling.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=15.0,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            error_gain=(MX106V2_ENCODER_COUNTS_PER_REV / (2 * np.pi))
            / (MX106V2_KP_DIVISOR * MX106V2_PWM_LIMIT),
            max_pwm=1.0,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 5.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


MX64V2_ENCODER_COUNTS_PER_REV = 4096  # Resolution: 4096 pulse/rev (datasheet)
MX64V2_KP_DIVISOR = 141.32  # Empirically measured with bam/dynamixel/configure_kp_divisor.py
# (pooled least-squares estimate over 4425 samples, --duration 15, kp in
# [100, 200, 300, 400, 600, 800]; per-kp estimates in [140.77, 142.17], i.e.
# +/-0.7% -- tight and consistent, distinct from MX106V2_KP_DIVISOR (~147):
# MX-64(2.0) (model 311) and MX-106(2.0) (model 321) are different physical
# actuators, so a different real divisor is expected. An earlier short-
# duration run (--duration 4, the KPS default) on the same servo gave a
# noisier 137.96-151.59 spread pooling to 142.24 -- consistent with that
# run being under-sampled rather than the servo's real value drifting.
MX64V2_PWM_LIMIT = 885   # Present PWM limit, read back from the servo's own register (confirmed)


class MX64V2Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel MX-64(2.0) actuator, i.e. the Protocol 2.0
    firmware variant -- NOT the same as the "mx64" key elsewhere in this
    repo, which targets the original Protocol 1.0 MX-64 via MXActuator
    (oscilloscope-measured error_gain=0.158). Control table layout is
    identical to XH540 (Torque Enable=64, Position P/I/D Gain=84/82/80 with
    the same KPP=register/MX64V2_KP_DIVISOR conversion, Goal Position=116,
    Present PWM=124, PWM Limit=36), so bam/dynamixel/configure_kp_divisor.py
    and DynamixelXH540 work unmodified for it -- just point --port/--id at it.

    kt/R/armature below are carried over unchanged from MXActuator (same
    ranges used there for both MX-64 and MX-106); only error_gain was
    re-derived for this firmware's control table / KP scaling.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=15.0,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            error_gain=(MX64V2_ENCODER_COUNTS_PER_REV / (2 * np.pi))
            / (MX64V2_KP_DIVISOR * MX64V2_PWM_LIMIT),
            max_pwm=1.0,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(1.6, 1.0, 3.0)

        # Motor resistance [Ohm]
        self.model.R = Parameter(2.0, 1.0, 5.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


XH540_ENCODER_COUNTS_PER_REV = 4096  # Resolution: 4096 pulse/rev (datasheet)
XH540_KP_DIVISOR = 128  # Position P Gain: KPP = KPP(TBL) / 128 (datasheet, not empirically corrected)
XH540_PWM_LIMIT = 885   # Default Present PWM limit for XH540-W270 (datasheet)


class XHActuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XH-430 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin = 16.5,
            kp = 32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            error_gain=(XH540_ENCODER_COUNTS_PER_REV / (2 * np.pi))
            / (XH540_KP_DIVISOR * XH540_PWM_LIMIT),
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm = 1.0
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        # Center from datasheet stall torque/current @12V (9.9/4.9 ~= 2.02 Nm/A),
        # range widened to also cover the no-load-speed cross-check (~2.94 Nm/A)
        self.model.kt = Parameter(2.0, 0.6, 4.0)

        # Motor resistance [Ohm]
        # Stall condition U ~= I*R across 11.1/12.0/14.8V all agree on ~2.45-2.51 Ohm
        self.model.R = Parameter(2.48, 1.2, 5.0)

        # Motor armature / apparent inertia [kg m^2]
        # No rotor inertia published (coreless Maxon, part number undisclosed),
        # so this is left for the optimizer to identify
        self.model.armature = Parameter(0.001, 0.0001, 0.1)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
class XL320Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XL-320 actuator
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.5,
            kp=32.0,
            # This gain, if multiplied by a position error and firmware KP, gives duty cycle
            # It was determined using an oscilloscope and XL-320 actuators
            error_gain=0.05048199,
            # Maximum allowable duty cycle, also determined with oscilloscope
            max_pwm=1.0,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.7, 0.25, 1.5)

        # Motor resistance [Ohm]
        self.model.R = Parameter(5.0, 3.0, 40.0)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.0005, 0.0001, 0.01)

        self.model.max_load_friction = 1.0

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


XL330_ENCODER_COUNTS_PER_REV = 4096
XL330_KP_DIVISOR = 256  # Empirically observed for XL330 (manual mentions 128)
XL330_PWM_LIMIT = 885  # Default Present PWM limit for XL330


class XL330Actuator(VoltageControlledActuator):
    """
    Represents a Dynamixel XL330 actuator.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.5,
            kp=400,
            error_gain=(XL330_ENCODER_COUNTS_PER_REV / (2 * np.pi))
            / (XL330_KP_DIVISOR * XL330_PWM_LIMIT),
            max_pwm=1.0
        )

    def initialize(self):
        self.model.kt = Parameter(1.6, 0.1, 3.0)
        self.model.R = Parameter(2.6, 2.0, 5.0)
        self.model.armature = Parameter(0.005, 0.0001, 0.05)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class XL330CurrentActuator(CurrentControlledActuator):
    """
    Represents a Dynamixel XL330 actuator controlled in current position mode.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=7.5,
            kp=400,
            error_gain=0.01,
        )

    def initialize(self):
        self.model.kt = Parameter(1.6, 0.1, 3.0)
        self.model.R = Parameter(2.6, 2.0, 5.0)
        self.model.armature = Parameter(0.005, 0.0001, 0.05)
        self.model.current_limit = Parameter(1.5, 1.0, 3.0)

    def compute_torque(self, control, torque_enable, q, dq):
        """Torque from the regulated **bus** current (XL330-specific).

        Unlike the standard current-controlled servo (which senses and regulates
        the motor *phase* current, so ``torque = kt * current``), the XL330
        measures current on the *input/bus* side of the H-bridge. Its current
        loop therefore regulates the bus current, whose magnitude relates to the
        phase current through the PWM duty by power balance:

        .. math::

            I_\\text{bus} = \\text{duty} \\cdot I_\\text{phase},
            \\qquad I_\\text{phase} = \\frac{\\text{duty}\\,v_\\text{in} - k_t\\dot q}{R}

        Given the commanded bus current (``control``, signed by drive direction),
        we recover the duty that produces it — the positive root of
        ``vin*duty^2 - kt*dq*duty - R*|I_bus| = 0`` in the drive direction — clamp
        it to the PWM limit, and return ``torque = kt * I_phase``. This makes the
        delivered torque duty-dependent (``~sqrt(I_bus)`` at stall) rather than
        linear in the command, which is the behaviour observed in the data.

        .. warning::

            UNTESTED. The bus-current hypothesis is inferred from position-mode
            data (the ``|duty| ~ sqrt(|error|)`` relationship), not yet confirmed
            by a direct torque measurement, and this model has not been validated
            by a full identification against held-out xl330i data. The math is
            unit-tested for self-consistency and the numpy path runs, but the
            torch/mjlab path has not been executed on a real tensor backend.
            Treat the resulting parameters and simulations as provisional.
        """
        if control is None:
            return 0.0

        kt = self.model.kt.value
        R = self.model.R.value
        vin = self.vin

        # Solve the power-balance quadratic for the duty in the drive direction.
        # Uses the backend (numpy / torch) so it also runs under mjlab on tensors;
        # ``abs`` and ``**`` dispatch correctly for floats, ndarrays and tensors.
        sign = self.backend.sign(control)
        disc = (kt * dq) ** 2 + 4.0 * vin * R * abs(control)
        duty = (kt * dq + sign * disc**0.5) / (2.0 * vin)

        # PWM saturation (XL330 PWM Limit, full 885 range -> duty in [-1, 1]).
        duty = self.backend.clamp(duty, -1.0, 1.0)

        phase_current = (duty * vin - kt * dq) / R
        torque = kt * phase_current
        return torque * torque_enable

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
