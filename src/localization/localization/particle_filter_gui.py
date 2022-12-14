import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

from particle_filter import ParticleFilter


class ParticleFilterGUI(QtWidgets.QMainWindow):
    def __init__(self, filename, drawn_particles=200):
        super().__init__()

        self.setStyleSheet("background-color: yellow;")

        self.particle_filter = ParticleFilter(filename)

        self.map_scale = self.particle_filter.grid.scale
        self.drawn_particles = drawn_particles

        self.filter_width = self.particle_filter.grid.width
        self.filter_height = self.particle_filter.grid.height

        self.label = QtWidgets.QLabel()
        self.canvas = QtGui.QPixmap(self.filter_width * self.map_scale, self.filter_height * self.map_scale)
        self.label.setPixmap(self.canvas)
        self.setCentralWidget(self.label)
        self.update()

    def update(self):
        painter = QtGui.QPainter(self.label.pixmap())
        pen = QtGui.QPen()
        self.draw_occupancy_grid(painter, pen)
        self.draw_particles(painter, pen)
        self.draw_robot(painter, pen)
        # self.draw_obstacles(painter, pen)
        painter.end()

    def draw_occupancy_grid(self, painter, pen):
        self.draw_occupancy(painter, pen)
        self.draw_grid(painter, pen)

    def draw_grid(self, painter, pen):
        pen.setWidth(1)
        pen.setColor(QtGui.QColor('black'))
        painter.setPen(pen)
        for i in range(self.filter_width):
            painter.drawLine(i * self.map_scale, 0, i * self.map_scale, self.filter_height * self.map_scale)

        for i in range(self.filter_height):
            painter.drawLine(0, i * self.map_scale, self.filter_width * self.map_scale, i * self.map_scale)

    def draw_occupancy(self, painter, pen):
        for i in range(self.filter_width):
            for j in range(self.filter_height):
                if self.particle_filter.grid.occupancy_grid[i][j]:
                    painter.fillRect(int(i * self.map_scale), int(j * self.map_scale),
                                     int((i + 1) * self.map_scale), int((j + 1) * self.map_scale),
                                     QtGui.QColor('black'))
                else:

                    painter.fillRect(int(i * self.map_scale), int(j * self.map_scale),
                                     int((i + 1) * self.map_scale), int((j + 1) * self.map_scale),
                                     QtGui.QColor('white'))

    def draw_particles(self, painter, pen):
        pen.setWidth(5)
        pen.setColor(QtGui.QColor('red'))
        painter.setPen(pen)
        for n in range(min(len(self.particle_filter.particles), self.drawn_particles)):
            painter.drawPoint(
                int(self.particle_filter.particles[n].x * self.map_scale),  # x
                int(self.particle_filter.particles[n].y * self.map_scale)  # y
            )

    def draw_robot(self, painter, pen):
        pen.setWidth(50)
        pen.setColor(QtGui.QColor('green'))
        painter.setPen(pen)
        painter.drawPoint(int(self.particle_filter.robot.x * self.map_scale),
                          int(self.particle_filter.robot.y * self.map_scale))

    def draw_obstacles(self, painter, pen):
        pen.setWidth(5)
        pen.setColor(QtGui.QColor('blue'))
        painter.setPen(pen)
        for obstacle in self.particle_filter.grid.obstacles:
            painter.fillRect(int(obstacle[0] * self.map_scale), int(obstacle[1] * self.map_scale),
                             int(obstacle[2] * self.map_scale), int(obstacle[3] * self.map_scale),
                             QtGui.QColor('blue'))

    def draw_markers(self, painter, pen):
        pen.setWidth(5)
        pen.setColor(QtGui.QColor('blue'))
        painter.setPen(pen)
        for marker in self.particle_filter.grid.markers:
            painter.fillRect(int(marker[0] * self.map_scale), int(marker[1] * self.map_scale),
                             int(marker[0] + 10 * self.map_scale), int(marker[1] + 10 * self.map_scale),
                             QtGui.QColor('blue'))


if __name__ == "__main__":
    # f = "/mnt/c/Users/ianso/Nextcloud/Source-Repo/Python/InHomeDelivery/src/localization/localization/maps
    # /map_arena.json"
    f = "maps/new_map.json"
    app = QtWidgets.QApplication(sys.argv)
    window = ParticleFilterGUI(f, drawn_particles=5000)
    window.show()
    app.exec_()
