## 1. Solution: Using **function**
**Topic:** With or without function?

<table>
  <tr>
    <th>With function</th>
    <th>Without function</th>
  </tr>
  <tr>
    <td>

```python
import sys
from PyQt5.QtWidgets import *

def window():
    app = QApplication(sys.argv)
    win = QDialog()

    width = 500
    height = 500

    # Lấy độ phân giải màn hình hiện tại
    screen = QDesktopWidget().screenGeometry()
    screen_width = screen.width()
    screen_height = screen.height()

    # Tính vị trí để căn giữa
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
</td> <td>
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QPushButton, QDesktopWidget

def window():
    app = QApplication(sys.argv)
    win = QDialog()
    win.resize(500, 500)  # Kích thước cửa sổ

    # Nút bấm
    b1 = QPushButton("Button1", win)
    b1.move(50, 20)

    # Căn giữa cửa sổ
    qr = win.frameGeometry()  # Lấy khung hình của cửa sổ
    cp = QDesktopWidget().availableGeometry().center()  # Tâm của màn hình
    qr.moveCenter(cp)
    win.move(qr.topLeft())  # Di chuyển cửa sổ đến vị trí mới

    win.setWindowTitle("PyQt5 - Centered Window")
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    window()
</td> </tr> </table> ```
