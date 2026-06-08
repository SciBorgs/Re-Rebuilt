package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.Test.*;
import static org.sciborgs1155.lib.UnitTestingUtil.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.turret.SimTurret;
import org.sciborgs1155.robot.turret.Turret;

public class TurretTest {
  private Turret turret;

  @BeforeEach
  public void setup() {
    setupTests();
    turret = new Turret(new SimTurret());
    run(turret.zero());
    fastForward(1);
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(turret);
  }

  @Test
  public void setVoltage() {
    runUnitTest(turret.goToTest(0));
  }
}
