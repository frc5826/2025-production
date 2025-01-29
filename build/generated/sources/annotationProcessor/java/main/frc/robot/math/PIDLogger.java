package frc.robot.math;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;

public class PIDLogger extends ClassSpecificLogger<PID> {
  private static final VarHandle $P;
  private static final VarHandle $I;
  private static final VarHandle $D;
  private static final VarHandle $output;
  private static final VarHandle $actualSupplier;
  private static final VarHandle $min_output;
  private static final VarHandle $max_output;
  private static final VarHandle $deadband;

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(PID.class, MethodHandles.lookup());
      $P = lookup.findVarHandle(PID.class, "P", double.class);
      $I = lookup.findVarHandle(PID.class, "I", double.class);
      $D = lookup.findVarHandle(PID.class, "D", double.class);
      $output = lookup.findVarHandle(PID.class, "output", double.class);
      $actualSupplier = lookup.findVarHandle(PID.class, "actualSupplier", java.util.function.DoubleSupplier.class);
      $min_output = lookup.findVarHandle(PID.class, "min_output", double.class);
      $max_output = lookup.findVarHandle(PID.class, "max_output", double.class);
      $deadband = lookup.findVarHandle(PID.class, "deadband", double.class);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("[EPILOGUE] Could not load private fields for logging!", e);
    }
  }

  public PIDLogger() {
    super(PID.class);
  }

  @Override
  public void update(EpilogueBackend backend, PID object) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("P", ((double) $P.get(object)));
      backend.log("I", ((double) $I.get(object)));
      backend.log("D", ((double) $D.get(object)));
      backend.log("previous_error", object.previous_error);
      backend.log("integral", object.integral);
      backend.log("setpoint", object.setpoint);
      backend.log("error", object.error);
      backend.log("output", ((double) $output.get(object)));
      backend.log("actualSupplier", ((java.util.function.DoubleSupplier) $actualSupplier.get(object)).getAsDouble());
      backend.log("min_output", ((double) $min_output.get(object)));
      backend.log("max_output", ((double) $max_output.get(object)));
      backend.log("deadband", ((double) $deadband.get(object)));
      backend.log("calculate", object.calculate());
      backend.log("getOutput", object.getOutput());
    }
  }
}
