"""
Neuron keychain case, v3 (parametric, CadQuery)
Board: Waveshare ESP32-S3-Touch-LCD-1.28. Dimensions taken from Waveshare's STEP:
  glass Ø38.51, top at z=+2.02 ; PCB Ø35.67, z -2.98..-1.97 ; back components to z=-6.38
  USB-C body x -4.8..4.8, y -21.8..-14.2, z -6.2..-2.1 (opening faces -y)
  battery socket x -17.8..-12.6, y -4.2..3.4 (board frame; appears on the right when viewed from the back)
  BOOT/RESET buttons y 8.9..12.25 at x = ±(8.35..13.55)
Battery: 803040 LiPo, measured 9 x 30 x 44 mm incl. wrapping (+ clearance), flat behind the board, long axis vertical,
         wires leaving one short end; groove runs up the right wall (viewed from the back) to the battery socket.

Two printed parts:
  bezel  : front ring. Holds the glass with a 1.4 mm lip, locates the PCB, 4 corner bosses with pilot holes.
  shell  : back. Battery pocket, USB-C slot, wire channel to the battery socket, keyring loop, 4 screw holes.
Assembly: drop board into bezel (glass first), battery into shell, close, 4 x M2 x 16 self-tapping from the back.
Run:  python neuron_case.py   -> out/bezel.stl, out/shell.stl, out/neuron_case.step, out/preview.svg
"""
import cadquery as cq
import os

# ---------------- parameters (mm) ----------------
WALL        = 2.0
CLEAR       = 0.30     # radial clearance around glass / PCB
OUT_W, OUT_H, OUT_R = 46.0, 50.0, 10.0        # shell footprint (x, y) and corner radius
GLASS_D     = 38.51
GLASS_TOP   = 2.02
LIP         = 1.4      # bezel overlap onto the glass edge (visible glass Ø = GLASS_D - 2*LIP = 35.7; active area is 33.4)
LIP_T       = 1.2      # bezel lip thickness above the glass
PCB_D       = 35.67
PCB_BOT     = -2.98
COMP_BOT    = -6.38    # lowest back component
SPLIT_Z     = -3.2     # where bezel meets shell (just under the PCB)
TAB_W       = 17.0 + 2 * CLEAR   # the board is not round: glass/LCD/PCB carry a tab at the USB-C end
TAB_Y       = -22.0    # how far the tab slot reaches (board stack ends at -21.2, USB-C body at -21.8 is below the PCB)

BAT_L, BAT_W, BAT_T = 44.0, 30.0, 9.0          # measured, including wrapping and protection strip
BAT_CLEAR   = 0.5
BAT_TOP     = COMP_BOT - 0.6                   # battery face sits 0.6 under the tallest component
BAT_BOT     = BAT_TOP - BAT_T - BAT_CLEAR
FLOOR_Z     = BAT_BOT - WALL                   # outer back face
BAT_WIRE_SIDE = "right"                        # which side of the battery the wires leave: "left" or "right" (viewed from the back)

USB_W, USB_H = 10.0, 7.0                       # slot for a USB-C plug shell
USB_ZC      = -4.15

LOOP_D, LOOP_HOLE, LOOP_T = 10.0, 4.5, 4.0     # keyring loop
BOSS_D, BOSS_XY = 5.2, (18.0, 20.5)            # corner bosses, outside the battery footprint
SCREW_HOLE_SHELL = 2.3                         # clearance for M2
PILOT_BEZEL = 1.6                              # self-tap pilot in bezel

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

def rrect(w, h, r, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).rect(w, h).extrude(z1 - z0).edges("|Z").fillet(r)

boss_pts = [(sx * BOSS_XY[0], sy * BOSS_XY[1]) for sx in (-1, 1) for sy in (-1, 1)]

# ---------------- bezel ----------------
bez_top = GLASS_TOP + LIP_T
bezel = rrect(OUT_W, OUT_H, OUT_R, SPLIT_Z, bez_top)
# glass pocket (Ø glass + clearance) from the lip down to the split, plus the slot for the tab at the USB-C end
pocket_h = GLASS_TOP - SPLIT_Z
pocket = cq.Workplane("XY").workplane(offset=SPLIT_Z).circle(GLASS_D / 2 + CLEAR).extrude(pocket_h)
# the tab is a taper (measured from Waveshare's STEP, half-widths + 0.5 mm clearance)
TAB_PTS = [(-13.9, -13.5), (-11.9, -16.0), (-9.9, -18.0), (-8.7, -19.0), (-7.6, -20.0), (-7.0, -21.0), (-5.0, -21.6), (-5.0, -22.5),
           ( 5.0, -22.5), ( 5.0, -21.6), ( 7.0, -21.0), ( 7.6, -20.0), ( 8.7, -19.0), ( 9.9, -18.0), (11.9, -16.0), (13.9, -13.5)]
slot = cq.Workplane("XY").workplane(offset=SPLIT_Z).polyline(TAB_PTS).close().extrude(pocket_h)
bezel = bezel.cut(pocket.union(slot))
# visible opening through the lip
bezel = bezel.cut(cq.Workplane("XY").workplane(offset=GLASS_TOP - 0.01).circle(GLASS_D / 2 - LIP).extrude(LIP_T + 0.02))
# soften the outer front edge and the lip edge
bezel = bezel.faces(">Z").edges().fillet(1.0)
# corner bosses with pilot holes (part of the bezel, running down to the split)
bosses = cq.Workplane("XY").workplane(offset=SPLIT_Z).pushPoints(boss_pts).circle(BOSS_D / 2).extrude(GLASS_TOP - 0.5 - SPLIT_Z)
bezel = bezel.union(bosses)
bezel = bezel.cut(cq.Workplane("XY").workplane(offset=SPLIT_Z - 0.01).pushPoints(boss_pts).circle(PILOT_BEZEL / 2).extrude(5.0))

# ---------------- shell ----------------
shell = rrect(OUT_W, OUT_H, OUT_R, FLOOR_Z, SPLIT_Z)
shell = shell.faces("<Z").edges().fillet(1.0)          # soften the back edge while the body is still simple
cav_w, cav_h, cav_r = OUT_W - 2 * WALL, OUT_H - 2 * WALL, OUT_R - WALL
cavity = rrect(cav_w, cav_h, cav_r, BAT_BOT, SPLIT_Z + 0.01)
shell = shell.cut(cavity)
# battery pocket walls: thin ribs so the cell cannot slide (two short ribs each side)
rib_x = BAT_W / 2 + BAT_CLEAR / 2 + 0.6
wx = (1 if BAT_WIRE_SIDE == "right" else -1)
for yy in (-10.0, 10.0):                       # ribs on the side away from the wires
    rib = cq.Workplane("XY").workplane(offset=BAT_BOT).center(-wx * rib_x, yy).rect(1.2, 8.0).extrude(BAT_T + BAT_CLEAR - 0.5)
    shell = shell.union(rib)
# screw bosses from floor up to the split, with clearance holes through the floor
sb = cq.Workplane("XY").workplane(offset=FLOOR_Z).pushPoints(boss_pts).circle(BOSS_D / 2).extrude(SPLIT_Z - FLOOR_Z)
shell = shell.union(sb)
shell = shell.cut(cq.Workplane("XY").workplane(offset=FLOOR_Z - 0.01).pushPoints(boss_pts).circle(SCREW_HOLE_SHELL / 2).extrude(SPLIT_Z - FLOOR_Z + 0.02))
# countersink-ish recess for the screw heads
shell = shell.cut(cq.Workplane("XY").workplane(offset=FLOOR_Z - 0.01).pushPoints(boss_pts).circle(2.1).extrude(1.0))
# USB-C slot in the -y wall
usb = cq.Workplane("XZ").workplane(offset=OUT_H / 2 - WALL - 0.5).center(0, USB_ZC).rect(USB_W, USB_H).extrude(WALL + 1.0).edges("|Y").fillet(1.5)
shell = shell.cut(usb)
# wire channel: a groove in the floor from the battery side to the socket side at mid height
chan = cq.Workplane("XY").workplane(offset=BAT_BOT - 0.01).center(wx * (BAT_W / 2 + 3.0), -10.0).rect(3.4, 26.0).extrude(1.6)
shell = shell.cut(chan)
# keyring loop at the top edge, integral, centred on the battery depth
loop_y = OUT_H / 2 + LOOP_D / 2 - 1.5
loop_zc = (FLOOR_Z + SPLIT_Z) / 2
loop = (cq.Workplane("XY").workplane(offset=loop_zc - LOOP_T / 2).center(0, loop_y)
        .circle(LOOP_D / 2).extrude(LOOP_T))
loop = loop.cut(cq.Workplane("XY").workplane(offset=loop_zc - LOOP_T / 2 - 0.01).center(0, loop_y).circle(LOOP_HOLE / 2).extrude(LOOP_T + 0.02))
neck = cq.Workplane("XY").workplane(offset=loop_zc - LOOP_T / 2).center(0, OUT_H / 2 - 1).rect(LOOP_D - 2, 5.0).extrude(LOOP_T)
shell = shell.union(loop).union(neck)

# ---------------- reference bodies (not printed) ----------------
glass = cq.Workplane("XY").workplane(offset=-2.6).circle(GLASS_D / 2).extrude(GLASS_TOP + 2.6)
pcb = cq.Workplane("XY").workplane(offset=PCB_BOT).circle(PCB_D / 2).extrude(1.0)
battery = cq.Workplane("XY").workplane(offset=BAT_TOP - BAT_T).rect(BAT_W, BAT_L).extrude(BAT_T)

# ---------------- export ----------------
cq.exporters.export(bezel, os.path.join(OUT, "bezel.stl"), tolerance=0.02, angularTolerance=0.1)
cq.exporters.export(shell, os.path.join(OUT, "shell.stl"), tolerance=0.02, angularTolerance=0.1)
asm = cq.Assembly()
asm.add(bezel, name="bezel", color=cq.Color(0.75, 0.9, 0.7))
asm.add(shell, name="shell", color=cq.Color(0.75, 0.9, 0.7))
asm.add(glass, name="glass_ref", color=cq.Color(0.2, 0.2, 0.25, 0.6))
asm.add(pcb, name="pcb_ref", color=cq.Color(0.1, 0.3, 0.5, 0.6))
asm.add(battery, name="battery_ref", color=cq.Color(0.8, 0.7, 0.2, 0.6))
asm.save(os.path.join(OUT, "neuron_case.step"))

# exploded preview, two views
ex = (cq.Workplane("XY").add(bezel.translate((0, 0, 12))).add(shell.translate((0, 0, -4)))
      .add(glass).add(pcb).add(battery))
cq.exporters.export(ex, os.path.join(OUT, "preview_iso.svg"),
                    opt={"projectionDir": (1, -1, 0.8), "width": 900, "height": 700, "showHidden": False, "strokeWidth": 0.6})
cq.exporters.export(ex, os.path.join(OUT, "preview_side.svg"),
                    opt={"projectionDir": (1, 0, 0), "width": 900, "height": 700, "showHidden": False, "strokeWidth": 0.6})

print("outer footprint %.1f x %.1f mm, total thickness %.1f mm" % (OUT_W, OUT_H, bez_top - FLOOR_Z))
print("battery pocket %.1f x %.1f x %.1f, floor at z=%.2f" % (BAT_W + BAT_CLEAR, cav_h, BAT_T + BAT_CLEAR, FLOOR_Z))
print("files in", OUT)
