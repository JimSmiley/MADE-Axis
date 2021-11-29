using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using extOSC;  //https://github.com/Iam1337/extOSC


public class MadeAxisOscSend : MonoBehaviour
{
    public int sliderNum;
    public int value;
    public bool testWithKeystrokes;
    MadeAxisOscSend(string axisName)
    {
        MadeAxisName = axisName;
    }
    OSCTransmitter transmitter;
    public string MadeAxisName = "/";
  
    void Start()
    {
        GameObject oscManager = GameObject.Find("OSC Manager");
        transmitter = oscManager.GetComponent<OSCTransmitter>();
    }
   void Update()       // Use this for testing only - delete once happy  
    {
        if (testWithKeystrokes)
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                HapticPulse(sliderNum, 3, value);
            }
            if (Input.GetKeyDown(KeyCode.S))
            {
                SnapTo(sliderNum, value);
            }
            if (Input.GetKeyDown(KeyCode.D))
            {
                Stepped(sliderNum, value);
            }
            if (Input.GetKeyDown(KeyCode.J))
            {
                Joystick(sliderNum, value);
            }
            if (Input.GetKeyDown(KeyCode.F))
            {
                FollowMode(sliderNum, value);
            }

            if (Input.GetKeyDown(KeyCode.L))
            {
                SetLed(value);
            }
        }
    }
    public void HapticPulse(int slider, int period, int value )  
    {
        var message = new OSCMessage(MadeAxisName + "/haptic");
        message.AddValue(OSCValue.Int(slider));  // 1 or 2
        message.AddValue(OSCValue.Int(period)); // suggest 3-5 ms
        message.AddValue(OSCValue.Int(value));  //0 -255, but 100 up is better
        transmitter.Send(message);
    }
    public void SnapTo(int slider, int value)
    {
        var message = new OSCMessage(MadeAxisName + "/snap");
        message.AddValue(OSCValue.Int(slider));  // 1 or 2 
        message.AddValue(OSCValue.Int(value));  //0  - 255
        transmitter.Send(message);
    }
    public void Stepped(int slider, int distance)  // set distance to 0 to turn off
    {
        var message = new OSCMessage(MadeAxisName + "/stepped");
        message.AddValue(OSCValue.Int(slider));  // 1 or 2 
        message.AddValue(OSCValue.Int(distance));  //1  - 100
        transmitter.Send(message);

    }
    public void Joystick(int slider, int active)  // active is int not bool - had issues mixing tyes in messages
    {
        var message = new OSCMessage(MadeAxisName + "/joystick");
        message.AddValue(OSCValue.Int(slider));  // 1 or 2 
        message.AddValue(OSCValue.Int(active));  //0  off, anything else true;
        transmitter.Send(message);

    }
    public void FollowMode(int slider, int difference)
    {
        var message = new OSCMessage(MadeAxisName + "/followMode");
        message.AddValue(OSCValue.Int(slider));  // 1 or 2 
        message.AddValue(OSCValue.Int(difference));  //0  off, anything else true;
        transmitter.Send(message);
    }

    public void SetLed(int val)
    {
        var message = new OSCMessage(MadeAxisName + "/led");
        message.AddValue(OSCValue.Int(val));  //0 -255
        transmitter.Send(message);

    }
}
