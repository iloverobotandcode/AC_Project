<div style="display: flex; gap: 20px;">

<!-- Bảng 1: With function -->
<div style="flex: 1;">

### 🟦 With Function

| Code |
|------|
|```python
import sys
from PyQt5.QtWidgets import *

def window():
    app = QApplication(sys.argv)
    win = QDialog()

    width = 500
    height = 500

    screen = QDesktopWidget().screenGeometry()
    screen_width = screen.width()
    screen_height = screen.height()

    xpos = (screen_width - width) // 2
    ypos = (screen_height - height) // 2

    b1 = QPushButton("Button1", win)
    b1.move(50, 20)

    win.setGeometry(xpos, ypos, width, height)
    win.setWindowTitle("PyQt5 - Cách của bạn, nhưng đúng")
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    window()
```|

</div>

<!-- Bảng 2: Without function -->
<div style="flex: 1;">

### 🟩 Using `frameGeometry()`

| Code |
|------|
|```python
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QPushButton, QDesktopWidget

def window():
    app = QApplication(sys.argv)
    win = QDialog()
    win.resize(500, 500)

    b1 = QPushButton("Button1", win)
    b1.move(50, 20)

    qr = win.frameGeometry()
    cp = QDesktopWidget().availableGeometry().center()
    qr.moveCenter(cp)
    win.move(qr.topLeft())

    win.setWindowTitle("PyQt5 - Centered Window")
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    window()
```|

</div>

</div>
