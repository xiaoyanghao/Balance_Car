import QtQuick                  2.12
import QtQuick.Controls         1.2


Item {
    visible:    true
    id: joystick
   // width:      //192 * 4
  //  height:     //108 * 4
   // color:      "grey"

    property real _offset: leftStick.width/2

    signal qmlsendstick(real px,real py)

    JoystickThumbPad {
        id:                     leftStick
        anchors.leftMargin:     xPositionDelta  + _offset
        anchors.bottomMargin:   -yPositionDelta + _offset
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter:   parent.verticalCenter
        width:                   150
        height:                  150
        imageHeight:             20
    }

    ///--You can also use signals
    Timer {
        interval:   50          // 20Hz
        running:    true
        repeat:     true
        onTriggered: {
            joystick.qmlsendstick(leftStick.xAxis,leftStick.yAxis)
           // console.log("leftStick.xAxis",leftStick.xAxis)
           // console.log("leftStick.yAxis",leftStick.yAxis)
        }
    }
}
