// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cm_ColorCycle extends Command {
    private final LEDSubsystem m_LEDs;
      Color[] colors = {
        Color.kAliceBlue,
        Color.kAntiqueWhite,
        Color.kAqua,
        Color.kAquamarine,
        Color.kAzure,
        Color.kBeige,
        Color.kBisque,
        Color.kBlanchedAlmond,
        Color.kBlue,
        Color.kBlueViolet,
        Color.kBrown,
        Color.kBurlywood,
        Color.kCadetBlue,
        Color.kChartreuse,
        Color.kChocolate,
        Color.kCoral,
        Color.kCornflowerBlue,
        Color.kCornsilk,
        Color.kCrimson,
        Color.kCyan,
        Color.kDarkBlue,
        Color.kDarkCyan,
        Color.kDarkGoldenrod,
        Color.kDarkGray,
        Color.kDarkGreen,
        Color.kDarkKhaki,
        Color.kDarkMagenta,
        Color.kDarkOliveGreen,
        Color.kDarkOrange,
        Color.kDarkOrchid,
        Color.kDarkRed,
        Color.kDarkSalmon,
        Color.kDarkSeaGreen,
        Color.kDarkSlateBlue,
        Color.kDarkSlateGray,
        Color.kDarkTurquoise,
        Color.kDarkViolet,
        Color.kDeepPink,
        Color.kDeepSkyBlue,
        Color.kDenim,
        Color.kDimGray,
        Color.kDodgerBlue,
        Color.kFirebrick,
        Color.kFirstBlue,
        Color.kFirstRed,
        Color.kFloralWhite,
        Color.kForestGreen,
        Color.kFuchsia,
        Color.kGainsboro,
        Color.kGhostWhite,
        Color.kGold,
        Color.kGoldenrod,
        Color.kGray,
        Color.kGreen,
        Color.kGreenYellow,
        Color.kHoneydew,
        Color.kHotPink,
        Color.kIndianRed,
        Color.kIndigo,
        Color.kIvory,
        Color.kKhaki,
        Color.kLavender,
        Color.kLavenderBlush,
        Color.kLawnGreen,
        Color.kLemonChiffon,
        Color.kLightBlue,
        Color.kLightCoral,
        Color.kLightCyan,
        Color.kLightGoldenrodYellow,
        Color.kLightGray,
        Color.kLightGreen,
        Color.kLightPink,
        Color.kLightSalmon,
        Color.kLightSeaGreen,
        Color.kLightSkyBlue,
        Color.kLightSlateGray,
        Color.kLightSteelBlue,
        Color.kLightYellow,
        Color.kLime,
        Color.kLimeGreen,
        Color.kLinen,
        Color.kMagenta,
        Color.kMaroon,
        Color.kMediumAquamarine,
        Color.kMediumBlue,
        Color.kMediumOrchid,
        Color.kMediumPurple,
        Color.kMediumSeaGreen,
        Color.kMediumSlateBlue,
        Color.kMediumSpringGreen,
        Color.kMediumTurquoise,
        Color.kMediumVioletRed,
        Color.kMidnightBlue,
        Color.kMintcream,
        Color.kMistyRose,
        Color.kMoccasin,
        Color.kNavajoWhite,
        Color.kNavy,
        Color.kOldLace,
        Color.kOlive,
        Color.kOliveDrab,
        Color.kOrange,
        Color.kOrangeRed,
        Color.kOrchid,
        Color.kPaleGoldenrod,
        Color.kPaleGreen,
        Color.kPaleTurquoise,
        Color.kPaleVioletRed,
        Color.kPapayaWhip,
        Color.kPeachPuff,
        Color.kPeru,
        Color.kPink,
        Color.kPlum,
        Color.kPowderBlue,
        Color.kPurple,
        Color.kRed,
        Color.kRosyBrown,
        Color.kRoyalBlue,
        Color.kSaddleBrown,
        Color.kSalmon,
        Color.kSandyBrown,
        Color.kSeaGreen,
        Color.kSeashell,
        Color.kSienna,
        Color.kSilver,
        Color.kSkyBlue,
        Color.kSlateBlue,
        Color.kSlateGray,
        Color.kSnow,
        Color.kSpringGreen,
        Color.kSteelBlue,
        Color.kTan,
        Color.kTeal,
        Color.kThistle,
        Color.kTomato,
        Color.kTurquoise,
        Color.kViolet,
        Color.kWheat,
        Color.kWhite,
        Color.kWhiteSmoke,
        Color.kYellow,
        Color.kYellowGreen
      };

  /** Creates a new cm_ColorViewer. */
  public cm_ColorCycle(LEDSubsystem leds) {
    m_LEDs = leds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (Color color : colors) {
      m_LEDs.cm_setLedToColor(color).andThen(new WaitCommand(3));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
