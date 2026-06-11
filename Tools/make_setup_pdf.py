#!/usr/bin/env python3
"""Generate a Mission Planner / RC setup reference PDF for the LAT SILS uSTOL model."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

OUT = "LAT_SILS_MissionPlanner_RC_Setup.pdf"

ACCENT = "#1f5fb0"
HEADER_BG = "#1f5fb0"
HEADER_FG = "white"
ROW_ALT = "#eef3fb"

PAGE_W, PAGE_H = 8.27, 11.69  # A4 portrait (inches)


def new_page(pdf):
    fig = plt.figure(figsize=(PAGE_W, PAGE_H))
    fig.subplots_adjust(left=0.07, right=0.93, top=0.95, bottom=0.05)
    return fig


def heading(fig, y, text, size=15):
    fig.text(0.07, y, text, fontsize=size, fontweight="bold", color=ACCENT, va="top")
    fig.add_artist(plt.Line2D([0.07, 0.93], [y - 0.012, y - 0.012],
                              color=ACCENT, lw=1.2, transform=fig.transFigure))
    return y - 0.035


def para(fig, y, text, size=9.5, indent=0.07, gap=0.022, color="black"):
    fig.text(indent, y, text, fontsize=size, va="top", wrap=True, color=color)
    return y - gap


def table(fig, y, col_labels, rows, col_widths, height_per_row=0.030, fontsize=9):
    n = len(rows) + 1
    tbl_h = n * height_per_row
    ax = fig.add_axes([0.07, y - tbl_h, 0.86, tbl_h])
    ax.axis("off")
    t = ax.table(cellText=rows, colLabels=col_labels, colWidths=col_widths,
                 loc="upper left", cellLoc="left")
    t.auto_set_font_size(False)
    t.set_fontsize(fontsize)
    t.scale(1, 1.4)
    for (r, c), cell in t.get_celld().items():
        cell.set_edgecolor("#c5cdd8")
        cell.PAD = 0.03
        if r == 0:
            cell.set_facecolor(HEADER_BG)
            cell.set_text_props(color=HEADER_FG, fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor(ROW_ALT)
    return y - tbl_h - 0.02


# ----------------------------------------------------------------------------
with PdfPages(OUT) as pdf:

    # ===================== PAGE 1 =====================
    fig = new_page(fig=None) if False else new_page(pdf)
    fig.text(0.07, 0.97, "LAT SILS — Mission Planner & RC Setup Reference",
             fontsize=18, fontweight="bold", color=ACCENT, va="top")
    fig.text(0.07, 0.935, "Active FDM model: PLANE_USTOL_V1  (LAT_SIM_Runner.cpp:25)  •  Flight mode: MANUAL",
             fontsize=10, va="top", color="#444444")

    y = 0.90
    y = heading(fig, y, "1.  How the control chain works")
    y = para(fig, y,
             "In MANUAL mode ArduPlane passes pilot input straight through to the servo outputs. The full chain is:")
    y = para(fig, y,
             "   gamepad axis  →  RC channel  →  RCMAP (roll/pitch/thr/yaw)  →  MANUAL pass-through  →  SERVOn_FUNCTION  →  FDM surface",
             size=9, color=ACCENT)
    y = para(fig, y,
             "So two independent mappings must agree: (A) the SERVO output functions the FDM reads, and")
    y = para(fig, y,
             "(B) the joystick-axis → RC-channel binding in Mission Planner. Both are documented below.")

    y -= 0.015
    y = heading(fig, y, "2.  Servo OUTPUT function mapping  (SERVOn_FUNCTION)")
    y = para(fig, y,
             "These tell ArduPlane which output channel carries which control. The uSTOL FDM reads surfaces")
    y = para(fig, y,
             "from SERVO10–13 and motors from SERVO1–9. Set these params (changed from stock ArduPlane):")
    y -= 0.005
    rows = [
        ["SERVO1 – SERVO9", "Throttle", "70", "18 EDF motors (2 EDFs per channel)"],
        ["SERVO10",         "Aileron",  "4",  "Roll"],
        ["SERVO11",         "Elevator", "19", "Pitch"],
        ["SERVO12",         "Rudder",   "21", "Yaw"],
        ["SERVO13",         "Flap",     "2",  "Flaps (manual input)"],
    ]
    y = table(fig, y, ["Output channel", "Function", "Value", "Controls"],
              rows, col_widths=[0.26, 0.18, 0.12, 0.44])

    y -= 0.005
    y = heading(fig, y, "3.  RC INPUT → joystick axis mapping  (Mission Planner Joystick screen)")
    y = para(fig, y,
             "RC1–RC4 are fixed as roll/pitch/throttle/yaw by RCMAP defaults (1/2/3/4). On this screen you only")
    y = para(fig, y,
             "bind each gamepad axis to a row. Recommended Mode-2 layout for the Logitech Dual Action:")
    y -= 0.005
    rows = [
        ["RC1", "Roll",     "Z",  "Right stick  L/R", "test"],
        ["RC2", "Pitch",    "Rz", "Right stick  U/D", "likely YES"],
        ["RC3", "Throttle", "Y",  "Left stick   U/D", "likely YES"],
        ["RC4", "Yaw",      "X",  "Left stick   L/R", "test"],
        ["RC5", "Flap (input)", "button / spare axis", "—", "—"],
    ]
    y = table(fig, y, ["RC ch", "Function", "Gamepad axis", "Physical stick", "Reverse?"],
              rows, col_widths=[0.10, 0.18, 0.24, 0.28, 0.14])

    y = para(fig, y,
             "Left stick = X (L/R) & Y (U/D);  Right stick = Z (L/R) & Rz (U/D). Set RC5/RC6 you don't use to None.",
             size=9, color="#444444")

    fig.savefig('/tmp/setup_page1.png', dpi=110)
    pdf.savefig(fig)
    plt.close(fig)

    # ===================== PAGE 2 =====================
    fig = new_page(pdf)
    y = 0.95
    y = heading(fig, y, "4.  Flap setup")
    y = para(fig, y, "Two separate 'channels' are involved — don't confuse them:")
    y = para(fig, y, "   • OUTPUT channel = SERVO13  (SERVO13_FUNCTION = 2 'Flap'), which the FDM reads as delta_f.")
    y = para(fig, y, "   • INPUT channel  = a SPARE RC channel that commands the flap. RC1–4 are taken, so use RC5.")
    y -= 0.005
    y = para(fig, y, "Manual flap (stick / switch driven):")
    rows = [
        ["SERVO13_FUNCTION", "2", "Output = manual Flap"],
        ["FLAP_IN_CHANNEL",  "5", "RC5 is the manual flap input channel"],
    ]
    y = table(fig, y, ["Parameter", "Value", "Meaning"], rows,
              col_widths=[0.30, 0.12, 0.44])
    y = para(fig, y, "Then bind RC5 to a gamepad button or spare axis on the Joystick screen.")
    y -= 0.01
    y = para(fig, y, "Automatic (speed-scheduled) flap — no RC channel needed:")
    rows = [
        ["SERVO13_FUNCTION", "3",  "Output = Flap_auto"],
        ["FLAP_1_SPEED",     "—",  "Speed (m/s) at/below which FLAP_1_PERCNT is applied"],
        ["FLAP_1_PERCNT",    "—",  "Flap deflection % at FLAP_1_SPEED"],
        ["FLAP_2_SPEED",     "—",  "Lower speed threshold for second flap step"],
        ["FLAP_2_PERCNT",    "—",  "Flap deflection % at FLAP_2_SPEED"],
    ]
    y = table(fig, y, ["Parameter", "Value", "Meaning"], rows,
              col_widths=[0.26, 0.12, 0.48])

    y -= 0.01
    y = heading(fig, y, "5.  Verifying the mapping (and setting the Reverse boxes)")
    y = para(fig, y, "Never guess the Reverse checkboxes — confirm each control live:")
    y = para(fig, y, "   1.  Build & launch SITL, set mode to MANUAL, arm.")
    y = para(fig, y, "   2.  Move each stick and watch the surface in the sim / the SERVO output bars.")
    y = para(fig, y, "   3.  Correct ArduPlane sense:")
    y = para(fig, y, "          • stick RIGHT      → right roll (aileron)")
    y = para(fig, y, "          • PULL BACK        → nose UP (elevator)")
    y = para(fig, y, "          • throttle stick UP→ throttle increases")
    y = para(fig, y, "          • yaw RIGHT        → nose yaws right (rudder)")
    y = para(fig, y, "   4.  Any control that moves backwards → tick that row's Reverse box and click Save.")
    y = para(fig, y, "Gamepad Y / Rz axes usually read 'up = low', which is why throttle and pitch typically need Reverse.",
             color="#444444")

    y -= 0.01
    y = heading(fig, y, "6.  ArduPlane SERVO function-number reference")
    rows = [
        ["0", "Disabled"], ["2", "Flap (manual)"], ["3", "Flap_auto"],
        ["4", "Aileron"], ["19", "Elevator"], ["21", "Rudder"], ["70", "Throttle"],
    ]
    y = table(fig, y, ["Value", "Function"], rows, col_widths=[0.14, 0.40])

    fig.savefig('/tmp/setup_page2.png', dpi=110)
    pdf.savefig(fig)
    plt.close(fig)

    # ===================== PAGE 3 =====================
    fig = new_page(pdf)
    y = 0.95
    y = heading(fig, y, "7.  Important caveats")
    y = para(fig, y, "• The active FDM is HARD-CODED at LAT_SIM_Runner.cpp:25  →  vehcle.plane_model = PLANE_USTOL_V1.")
    y = para(fig, y, "  This entire mapping is specific to uSTOL. Other models read DIFFERENT channels:")
    y -= 0.005
    rows = [
        ["PLANE_USTOL_V1",   "SERVO10", "SERVO11", "SERVO12", "SERVO13", "SERVO1–9"],
        ["PLANE_EQX / DEFAULT", "SERVO10/11", "SERVO13", "SERVO6", "SERVO12", "SERVO1–4"],
    ]
    y = table(fig, y, ["Model", "Aileron", "Elevator", "Rudder", "Flap", "Throttle/motors"],
              rows, col_widths=[0.24, 0.16, 0.14, 0.13, 0.13, 0.18], fontsize=8.5)
    y = para(fig, y, "  The existing Latest.parm matches the EQX/DEFAULT layout — it will NOT work with uSTOL")
    y = para(fig, y, "  (elevator & rudder would be dead). To fly EQX/default instead: change the model at")
    y = para(fig, y, "  LAT_SIM_Runner.cpp:25, rebuild, and load Latest.parm.")

    y -= 0.01
    y = para(fig, y, "• PWM range / sign: the FDM maps PWM 1100→-max, 1900→+max, centred at 1500, with NON-reversed")
    y = para(fig, y, "  travel (sign baked into the model). ArduPlane defaults SERVOn_MIN/MAX/TRIM = 1000/2000/1500 and")
    y = para(fig, y, "  SERVOn_REVERSED = 0 work as-is; full stick simply saturates at the surface limit. Do not reverse")
    y = para(fig, y, "  channels in the params unless live testing shows a control is inverted.")

    y -= 0.01
    y = para(fig, y, "• RCMAP defaults (RC1=Roll, RC2=Pitch, RC3=Throttle, RC4=Yaw) are assumed. If RCMAP_* was changed,")
    y = para(fig, y, "  the RC-channel rows above shift accordingly.")

    y -= 0.015
    y = heading(fig, y, "8.  Ready-to-load parameter block (uSTOL, MANUAL flight)")
    block = (
        "SERVO1_FUNCTION,70\nSERVO2_FUNCTION,70\nSERVO3_FUNCTION,70\nSERVO4_FUNCTION,70\n"
        "SERVO5_FUNCTION,70\nSERVO6_FUNCTION,70\nSERVO7_FUNCTION,70\nSERVO8_FUNCTION,70\n"
        "SERVO9_FUNCTION,70\nSERVO10_FUNCTION,4\nSERVO11_FUNCTION,19\nSERVO12_FUNCTION,21\n"
        "SERVO13_FUNCTION,2\nFLAP_IN_CHANNEL,5"
    )
    fig.text(0.07, y, block, fontsize=9, family="monospace", va="top",
             bbox=dict(boxstyle="round,pad=0.6", facecolor="#f4f6f9", edgecolor="#c5cdd8"))

    fig.text(0.07, 0.05, "Generated for LAT SILS  •  branch: Sushanth  •  model: PLANE_USTOL_V1",
             fontsize=8, color="#888888", va="bottom")

    fig.savefig('/tmp/setup_page3.png', dpi=110)
    pdf.savefig(fig)
    plt.close(fig)

print("Wrote", OUT)
