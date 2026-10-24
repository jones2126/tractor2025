#!/usr/bin/env python3
"""Create the printable AREA-OVER-ROAD blades-off field-test checklist."""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    KeepTogether,
    PageTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parents[5]
OUTPUT = ROOT / "output/pdf/AREA-OVER-ROAD_blades-off_field_test_checklist_rev002.pdf"


def checkbox_rows(items, style):
    rows = [[Paragraph("<b>[&nbsp;&nbsp;]</b>", style), Paragraph(item, style)] for item in items]
    table = Table(rows, colWidths=[0.30 * inch, 6.72 * inch], hAlign="LEFT")
    table.setStyle(
        TableStyle(
            [
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 0),
                ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                ("TOPPADDING", (0, 0), (-1, -1), 1.7),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 1.7),
            ]
        )
    )
    return table


def section(title, items, heading, body):
    return KeepTogether([Paragraph(title, heading), checkbox_rows(items, body), Spacer(1, 0.07 * inch)])


def draw_page(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#CBD5E1"))
    canvas.line(0.48 * inch, 0.42 * inch, 8.02 * inch, 0.42 * inch)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(colors.HexColor("#475569"))
    canvas.drawString(0.48 * inch, 0.25 * inch, "AREA-OVER-ROAD - supervised blades-off field test")
    canvas.drawRightString(7.70 * inch, 0.25 * inch, f"Page {doc.page}")
    canvas.restoreState()


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    styles = getSampleStyleSheet()
    title = ParagraphStyle(
        "TitleCompact", parent=styles["Title"], fontName="Helvetica-Bold",
        fontSize=18, leading=20, textColor=colors.HexColor("#0F172A"),
        alignment=TA_CENTER, spaceAfter=3,
    )
    status = ParagraphStyle(
        "Status", parent=styles["Normal"], fontName="Helvetica-Bold",
        fontSize=10, leading=12, textColor=colors.HexColor("#B91C1C"),
        alignment=TA_CENTER, spaceAfter=7,
    )
    heading = ParagraphStyle(
        "Heading", parent=styles["Heading2"], fontName="Helvetica-Bold",
        fontSize=11, leading=13, textColor=colors.white, backColor=colors.HexColor("#1E3A5F"),
        borderPadding=(3, 5, 3, 5), spaceBefore=4, spaceAfter=4,
    )
    body = ParagraphStyle(
        "BodyCompact", parent=styles["BodyText"], fontName="Helvetica",
        fontSize=8.7, leading=10.7, textColor=colors.HexColor("#111827"),
        alignment=TA_LEFT,
    )
    small = ParagraphStyle(
        "Small", parent=body, fontSize=7.6, leading=9.2, textColor=colors.HexColor("#334155"),
    )
    warning = ParagraphStyle(
        "Warning", parent=body, fontName="Helvetica-Bold", fontSize=9.2, leading=11.2,
        textColor=colors.HexColor("#7F1D1D"), backColor=colors.HexColor("#FEF2F2"),
        borderColor=colors.HexColor("#DC2626"), borderWidth=1, borderPadding=7,
        spaceAfter=7,
    )

    doc = BaseDocTemplate(
        str(OUTPUT), pagesize=letter,
        leftMargin=0.48 * inch, rightMargin=0.48 * inch,
        topMargin=0.42 * inch, bottomMargin=0.50 * inch,
        title="AREA-OVER-ROAD Blades-Off Field Test Checklist",
        author="Tractor Robot Project",
        subject="Printable supervised field test sequence",
    )
    frame = Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height, id="main")
    doc.addPageTemplates([PageTemplate(id="checklist", frames=[frame], onPage=draw_page)])

    story = [
        Paragraph("AREA-OVER-ROAD FIELD TEST CHECKLIST", title),
        Paragraph("DRAFT - UNTESTED IN THE FIELD | AUTHORIZED: ONE SUPERVISED BLADES-OFF RUN", status),
        Paragraph(
            "<b>CONTROL HANDOFF:</b> START MISSION launches the logger and Pure Pursuit controller while the phone remains in PAUSE. "
            "The tractor will not drive in PAUSE. When the dashboard says READY FOR GUARDED AUTO and the route is clear, selecting AUTO "
            "gives Pure Pursuit motion authority. Phone PAUSE, MANUAL, and E-STOP remain the operator overrides.",
            warning,
        ),
    ]

    facts = [
        ["Mission", "AREA-OVER-ROAD controller route rev003"],
        ["Controller file", "62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt"],
        ["Planned run", "Forward only - 690.216 m - about 11 min 30 sec - 1.0 m/s"],
        ["Operator", "Al alongside tractor; phone ready for PAUSE, MANUAL, or E-STOP"],
        ["Date / start time", "____________________________"],
        ["Weather / notes", "____________________________________________________________"],
    ]
    facts_table = Table(facts, colWidths=[1.18 * inch, 5.84 * inch], hAlign="LEFT")
    facts_table.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.45, colors.HexColor("#94A3B8")),
        ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#E2E8F0")),
        ("FONTNAME", (0, 0), (0, -1), "Helvetica-Bold"),
        ("FONTNAME", (1, 0), (1, -1), "Helvetica"),
        ("FONTSIZE", (0, 0), (-1, -1), 8.1),
        ("LEADING", (0, 0), (-1, -1), 9.6),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    story += [facts_table, Spacer(1, 0.08 * inch)]

    story += [
        section("1 - STATIONARY SAFETY AND MACHINE CHECK", [
            "Tractor is stationary on safe ground; area is clear of people, animals, vehicles, and new obstacles.",
            "MOWER BLADES ARE OFF and will remain off for the entire feasibility test.",
            "Phone controller is open, connected, and visibly shows <b>PAUSE</b>. Physical E-STOP is immediately reachable.",
            "Transmission is neutral; steering and throttle controls show no unexpected command.",
            "Fuel, tractor battery, router/Starlink regulated supply, and low-voltage alarm are satisfactory.",
            "TractorField network, RTK base, Tractor01, Teensy bridge, and status LEDs are operating normally.",
        ], heading, body),
        section("2 - DASHBOARD, VOICE GUIDANCE, AND PREFLIGHT", [
            "On the Windows field laptop, use PowerShell to run <b>start_area_over_road_field_dashboard_20261024.ps1</b> from the mission package.",
            "Open the AREA-OVER-ROAD web dashboard. Confirm the exact mission name and 3,463-waypoint route are shown.",
            "Start voice guidance. Keep the phone controller visible; voice guidance is informational and does not replace it.",
            "Confirm RTK Fixed, fixed carrier heading, valid heading, baseline 0.80-1.30 m, and heading accuracy at or below 1 degree.",
            "Confirm telemetry is fresh, operator link is stable, no duplicate controller is running, and preflight reports PASS.",
            "Confirm dashboard Start is field-enabled for this exact blades-off package. If Start is disabled or any gate fails, <b>DO NOT CONTINUE</b>.",
        ], heading, body),
        section("3 - MANUAL DRIVE TO THE START", [
            "On the phone controller select <b>MANUAL</b>; verify the displayed mode changed to Manual.",
            "Drive manually to F-out, following the approved F-out/O-out access line toward the mission start.",
            "Stop at the dashboard-indicated start pose and select <b>PAUSE</b>. Confirm zero drive command and stationary tractor.",
            "Recheck RTK Fixed, heading, route alignment, start distance, and that the path ahead is clear.",
        ], heading, body),
        section("4 - GUARDED MISSION START", [
            "With the phone still in <b>PAUSE</b>, press <b>START MISSION</b> on the dashboard and accept only the blades-off confirmation for this route.",
            "Wait for the dashboard/controller to report ready for guarded Auto. The tractor must remain stationary in Pause.",
            "Stand clear of the tractor's immediate path while keeping PAUSE, MANUAL, and E-STOP instantly available.",
            "Select <b>AUTO</b> on the phone controller. Confirm motion begins forward along the displayed access path.",
        ], heading, body),
    ]

    story += [
        section("5 - SUPERVISE THE ENTIRE RUN", [
            "Walk alongside at a safe offset; never stand in front of the tractor or inside the upcoming turn envelope.",
            "Watch the entry turn, pole loop, three pole-side stripe bypasses, headland turns, and exit turn closely.",
            "Continuously monitor RTK Fixed, heading validity, mode, operator link, commanded/actual speed, waypoint progress, and dashboard warnings.",
            "Expect Pure Pursuit to round the waypoint path. Abort if actual clearance or tracking is not plainly safe - especially near the pole and right-side chute.",
        ], heading, body),
        Paragraph("IMMEDIATE ABORT - USE THE FIRST SAFE CONTROL THAT FITS THE HAZARD", heading),
    ]
    abort_data = [
        [Paragraph("<b>MODE + PAUSE</b>", body), Paragraph("Normal first response: stop autonomous drive while retaining steering hold.", body)],
        [Paragraph("<b>MANUAL</b>", body), Paragraph("Use only when it is safe to take control and drive away or reposition forward.", body)],
        [Paragraph("<b>E-STOP</b>", body), Paragraph("Use immediately for imminent collision, unexpected acceleration/steering, lost control authority, or danger to a person.", body)],
    ]
    abort_table = Table(abort_data, colWidths=[1.35 * inch, 5.67 * inch], hAlign="LEFT")
    abort_table.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.7, colors.HexColor("#B91C1C")),
        ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#FEE2E2")),
        ("FONTNAME", (0, 0), (0, -1), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, -1), 8.5),
        ("LEADING", (0, 0), (-1, -1), 10),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    story += [abort_table, Spacer(1, 0.05 * inch), checkbox_rows([
        "Abort for a person/animal/vehicle, new obstacle, unsafe clearance, excessive tracking error, wrong mode, stale telemetry, loss of RTK Fixed/heading, unstable operator link, mechanical/electrical warning, or anything unexpected.",
        "After an abort, keep Pause selected. Do not resume automatically. Inspect the situation and record the approximate mission time/waypoint.",
        "Do not reverse with the deck engaged. For this test the deck remains disengaged throughout.",
    ], body), Spacer(1, 0.07 * inch)]

    story += [
        section("6 - NORMAL COMPLETION AND PARKING", [
            "Confirm the dashboard reports mission complete/goal reached and commanded speed is zero.",
            "Select <b>PAUSE</b>, then select <b>MANUAL</b> on the phone controller. Verify the displayed mode each time.",
            "Drive manually to the parking position. Stop, select PAUSE, set the parking brake, and turn off the engine.",
            "Stop/close the mission controller and dashboard only after the tractor is safely parked and motionless.",
        ], heading, body),
        section("7 - TELEMETRY AND END-OF-SESSION EVIDENCE", [
            "Run <b>finish_area_over_road_session_20261024.sh</b>. It must confirm that no Pure Pursuit controller remains running.",
            "Record the generated `/home/al/tractor01_area_over_road_*.tgz` archive path and SHA-256 value below.",
            "Copy the archive off Tractor01 and verify the copied file has the same SHA-256 hash before powering Tractor01 down.",
            "Download the original system log from the <b>Archer AXE75</b> at the RTK base station for the <b>TractorField</b> network.",
            "Record the AXE75 clock/time zone and router-log download time; preserve the original router log unchanged.",
        ], heading, body),
        Paragraph("FIELD RECORD", heading),
    ]
    record_rows = [
        ["End time", "________________________", "Result", "[  ] Complete   [  ] Aborted"],
        ["Abort time / waypoint", "________________________", "Reason", "____________________________"],
        ["Telemetry archive", "______________________________________________________________", "", ""],
        ["Archive SHA-256", "______________________________________________________________", "", ""],
        ["Router log filename", "______________________________________________________________", "", ""],
        ["AXE75 time zone / download", "______________________________________________________________", "", ""],
        ["Tracking / clearance notes", "______________________________________________________________", "", ""],
        ["", "______________________________________________________________", "", ""],
        ["Operator signature", "________________________", "Date", "____________________________"],
    ]
    record = Table(record_rows, colWidths=[1.62 * inch, 2.74 * inch, 0.62 * inch, 2.04 * inch], hAlign="LEFT")
    record.setStyle(TableStyle([
        ("SPAN", (1, 2), (3, 2)), ("SPAN", (1, 3), (3, 3)),
        ("SPAN", (1, 4), (3, 4)), ("SPAN", (1, 5), (3, 5)),
        ("SPAN", (1, 6), (3, 6)), ("SPAN", (1, 7), (3, 7)),
        ("GRID", (0, 0), (-1, -1), 0.45, colors.HexColor("#94A3B8")),
        ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#E2E8F0")),
        ("BACKGROUND", (2, 0), (2, 1), colors.HexColor("#E2E8F0")),
        ("BACKGROUND", (2, 8), (2, 8), colors.HexColor("#E2E8F0")),
        ("FONTNAME", (0, 0), (-1, -1), "Helvetica"),
        ("FONTNAME", (0, 0), (0, -1), "Helvetica-Bold"),
        ("FONTNAME", (2, 0), (2, 1), "Helvetica-Bold"),
        ("FONTNAME", (2, 8), (2, 8), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, -1), 7.8),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
    ]))
    story += [record, Spacer(1, 0.05 * inch), Paragraph(
        "Do not interpret a completed run as mowing authorization. Review telemetry and observed clearances before changing the mission's DRAFT - UNTESTED IN THE FIELD status.",
        small,
    )]

    doc.build(story)
    print(OUTPUT)


if __name__ == "__main__":
    main()
