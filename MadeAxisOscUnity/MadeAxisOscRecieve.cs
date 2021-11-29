using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using extOSC;
public class MadeAxisOscRecieve : MonoBehaviour
{
    //Call the method from an instance of OSCReceiverEventMessage set to the name of this axis. 

    public int sliderOne;
    public int sliderTwo;
    public int rotary;
    public bool rotarypress;
    public bool buttonPress;

    public void GetAxisOSCValues(OSCMessage message)
    {
        
        if (message.Values.Count == 5)
        {
            sliderOne = message.Values[0].IntValue;
            sliderTwo = message.Values[1].IntValue;
            rotary = message.Values[2].IntValue;
            rotarypress = !message.Values[3].BoolValue;
            buttonPress = message.Values[4].BoolValue;

        }
        

    }
}
