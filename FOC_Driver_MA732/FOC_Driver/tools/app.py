# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets
import sys
from gui.main_window import MainWindow

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
 