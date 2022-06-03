import math
import xml.etree.cElementTree as et

from PIL import Image

import turtle

# import user settings from an XML file
settings_file = et.parse("settings.xml")

input_set = settings_file.find("input").attrib
material_set = settings_file.find("material").attrib
tool_set = settings_file.find("tool").attrib

class CNC:
    def __init__(self, safe_height):
        super.__init__(self)

        self.started = False
        self.dwell_ms = 1000

    def err(self, msg):
        print("ERROR - " + msg)
        return self.end()
    
    def start(self):
        self.started = True

        # set absolute positioning and the origin,
        # then get to a safe height and start spindle at max velocity
        return (f"G90; G92 X0 Y0 Z0;\n" +
                f"G1 F{tool_set.plunge_rate} Z{-tool_set.clear_height};\n" + 
                f"M3;\n")

    def end(self):
        self.started = False

        # stop spindle and end the program
        return "M5; M0;\n"

    def fly_to(self, pos):
        if not self.started: return err("tried to fly when not started")

        pr = float(tool_set["plunge_rate_mm_s"])
        fr = float(tool_set["fly_rate_mm_s"])
        ch = float(tool_set["clear_height_mm"])
        dm = self.dwell_ms

        # move up to the safe height, move to pos, wait a bit, then move back down
        return (f"G1 F{pr} Z{-ch};\n" +
                f"G0 F{fr} X{pos[0]} Y{pos[1]};\n" +
                f"G4 P{dm};\n" + 
                f"G1 F{pr} Z0;\n")

    def cut_path(self, path, z_depth):
        if not self.started: return err("tried to cut when not started")

        pr = float(tool_set["plunge_rate_mm_s"])
        cr = float(tool_set["cut_rate_mm_s"])
        zd = float(material_set["z_depth_mm"])

        if z_depth > zd or z_depth < 0:
            return err("tried to cut outside material depth")

        cmd = f"G1 F{pr} Z{z_depth};\n"
    
        for pos in path:
            cmd += f"G1 F{cr} X{pos[0]} Y{pos[1]};\n"

        return cmd

    def cut_layers(self, path, z_depth):
        cmd = self.fly_to(path[0])

        pos_z = 0

        for pos_z in range(0, z_depth, tool_set.pass_depth):
            cmd += self.cut_path(pos_z, path)

        if pos_z < z_depth:
            cmd += self.cut_path(z_depth, path)

        return cmd

class PathMaker:
    def __init__(self):
        self.used_px = []
        self.paths = []
        self.depths = []

        # import raw pixel data from input path
        img = Image.open(input_set["path"])
        self.pixels = img.load()
        self.bounds = img.size

        pm = float(input_set["px_per_mm"])
        di = float(tool_set["diameter_mm"])

        self.rad_px = di * pm / 2

        self.step_px = 5
        self.step_ang = 65

        #turtle debugging
        turtle.delay(0)
        turtle.color('red')
        turtle.bgpic(input_set["path"])

        turtle.penup()
        turtle.setpos(-self.bounds[0]/2, self.bounds[1]/2)
        turtle.pendown()

    def confined(self, x_px, y_px):
        return (x_px >= 0 and x_px < self.bounds[0] and 
                y_px >= 0 and y_px < self.bounds[1])

    def inside(self, x_px, y_px, color):
        count = 0
        sum_x, sum_y = 0, 0

        for ang in range(0, 360):
            rx = math.cos(math.radians(ang)) * self.rad_px
            ry = math.sin(math.radians(ang)) * self.rad_px

            tx = x_px + rx
            ty = y_px + ry

            if self.confined(tx, ty) and self.pixels[tx, ty] == color:
                count += 1
                sum_x += rx
                sum_y += ry

        # normalize vx and vy
        mag = math.sqrt(pow(sum_x, 2) + pow(sum_y, 2))
        if mag == 0: mag = 1

        result = {
            "pct" : (count / 360) + 0.1,
            "dir" : (sum_x / mag, sum_y / mag),
        }

        return result

    def addPathNode(self, x_px, y_px):

        last_path =  self.paths[len(self.paths)-1]

        last_path.append((x_px, y_px))

        if len(last_path) <= 2: return

        lshift = 0
        
        for b in range(0, len(last_path)):
            dx = x_px - last_path[-b][0]
            dy = y_px - last_path[-b][1]
            mag = math.sqrt(pow(dx, 2) + pow(dy, 2))

            if mag > self.rad_px * 2:
                lshift = b
                break

        if lshift == 0: return

        # clear all pixels in the radius to transparent
        # so they won't be used again
        lx, ly = last_path[-lshift]

        for ang in range(0, 180):
            rx = round(lx + math.cos(math.radians(ang)) * self.rad_px)
            ry = round(ly + math.sin(math.radians(ang)) * self.rad_px)

            for column_y in range(-ry, ry+1):
                if self.confined(rx, column_y):
                    self.pixels[rx, column_y] = (0, 0, 0, 0)


    def makePath(self, x_px, y_px, color):
        # calculate how far into the mat this path should cut
        # based on the darkness of the color
        color_avg = color[0] + color[1] + color[2] / (3 * 255) 
            
        # don't make a path if the color is transparent/white or
        # a path already exists here
        if color[3] == 0 or color_avg == 1: return False

        # start making the new path
        self.paths.append([])
        self.depths.append(1 - color_avg)

        fails = 0

        while fails < 180:
            turtle.setpos(x_px - self.bounds[0]/2, -y_px + self.bounds[1]/2)

            res = self.inside(x_px, y_px, color)

            if res["pct"] < 1:
                x_px += res["dir"][0] * (1-res["pct"]) * self.rad_px * 2
                y_px += res["dir"][1] * (1-res["pct"]) * self.rad_px * 2
                
                d = res["dir"]

                res = self.inside(x_px, y_px, color)

                if res["pct"] >= 1:
                    self.addPathNode(x_px, y_px)

                    turtle.setpos(x_px - self.bounds[0]/2, -y_px + self.bounds[1]/2)
                    turtle.dot(self.rad_px*2, "blue")

                    # continue at a positive angle to the inside vector
                    # this will result in CW movement around the part
                    nang = math.atan2(d[1], d[0]) + math.radians(self.step_ang)
                    x_px -= math.cos(nang) * self.step_px
                    y_px -= math.sin(nang) * self.step_px

                else: fails += 1

            else: fails += 1

        return True

    def execute(self):
        for x_px in range(0, self.bounds[0]):
            for y_px in range(0, self.bounds[1], int(self.rad_px)):
                if self.makePath(x_px, y_px, self.pixels[x_px, y_px]):
                    while True: turtle.setpos(0, 0)


    def export(self):
        # optimize paths so that all straight lines consist of 2 points only
        paths_exp = self.paths

        l_node = None
        slope = None

        for path in paths_exp:
            path_cp = path[:]

            for node in path:
                if not l_node is None:
                    new_slope = (node[0] - l_node[0], node[1] - l_node[1])
                   
                    if new_slope == slope:
                        path_cp.remove(l_node)

                    slope = new_slope

                l_node = node

            path = path_cp

        # convert all points to millimeters
        for path in paths_exp:
            for node in path:
                node = node / float(input_set["px_per_mm"])

        return paths_exp, self.depths

pm = PathMaker()
pm.execute()

paths, depths = pm.export()

cnc = CNC()

# output the GCode to a file
output_file = open(input_set["path"]+".gcode", "w")

output_file.write(cnc.start())

for i in range(0, len(paths)):
    output_file.write(cnc.cut_layers(paths[i], depths[i]))

output_file.write(cnc.end())

output_file.close()

