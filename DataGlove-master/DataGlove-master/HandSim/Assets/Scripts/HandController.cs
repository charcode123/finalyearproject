using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Net.Sockets;
using System.Net;
using System.Text;
using System;

public class HandController : MonoBehaviour
{
    public struct UdpState
    {
        public UdpClient u;
        public IPEndPoint e;
    }

    public struct Finger
    {
        public Transform[] joints;
    }

    // Mutex for latest glove data
    readonly object gloveDataLock = new object();
    byte[] gloveData;
    Boolean received = false;

    public Transform wrist;
    public Finger[] fingers = new Finger[5];
    public Transform[] fingerRoots;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < 5; i++)
        {
            fingers[i].joints = new Transform[3];
            fingers[i].joints[0] = fingerRoots[i].Find("MetacarpophalangealJoint");
            fingers[i].joints[1] = fingers[i].joints[0].Find("Proximal/ProximalinterphalangealJoint");
            fingers[i].joints[2] = fingers[i].joints[1].Find("Intermediate/DistalinterphalangealJoint");
            if (!fingers[i].joints[0] || !fingers[i].joints[1] || !fingers[i].joints[2])
            {
                Debug.Log($"Error finding joints: Finger {i}");
            }
        }

        // Receive a message and write it to the console.
        IPEndPoint e = new IPEndPoint(IPAddress.Any, 5433);
        UdpClient u = new UdpClient(e);

        UdpState s = new UdpState();
        s.e = e;
        s.u = u;

        u.BeginReceive(new AsyncCallback(OnReceive), s);
        Debug.Log("Running...");
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Boolean shouldUpdate = true;
        lock(gloveDataLock)
        {
            if(!received)
                shouldUpdate = false;
        }

        if(shouldUpdate)
        {
            try
            {
                HandPose pose = ParseGloveData();

                wrist.localRotation = ToFrameOfReference((Quaternion)pose.wrist);
                for (int i=0; i < 5; i++)
                    for (int j=0; j < 3; j++)
                        fingers[i].joints[j].localRotation = ToFrameOfReference((Quaternion)pose.fingers[i].joints[j]);
            }
            catch (System.Exception)
            {
                Debug.Log($"Last data: {Encoding.ASCII.GetString(gloveData)}");
                throw;
            }
        }
    }

    private void OnReceive(IAsyncResult ar)
    {
        UdpState state = (UdpState)(ar.AsyncState);
        UdpClient client = state.u;
        IPEndPoint endpoint = state.e;

        lock (gloveDataLock)
        {
            gloveData = client.EndReceive(ar, ref endpoint);
            string receiveString = Encoding.ASCII.GetString(gloveData);
            if(receiveString[0] != '{')
                Debug.Log(receiveString);
            else
                received = true;
        }

        client.BeginReceive(new AsyncCallback(OnReceive), state);
    }

    private HandPose ParseGloveData()
    {
        byte[] rawData;
        lock (gloveDataLock)
        {
            rawData = (byte[])gloveData.Clone();
            received = false;
        }
        string strJSON = Encoding.ASCII.GetString(rawData);
        return JsonUtility.FromJson<HandPose>(strJSON);
    }

    private Quaternion ToFrameOfReference(Quaternion q)
    {
        return new Quaternion(-q.y, q.z, -q.x, q.w);
    }
}

[Serializable]
public struct FingerPose
{
    public SQuaternion[] joints;
}

[Serializable]
public struct HandPose
{
    public SQuaternion wrist;
    public FingerPose[] fingers;
    public String debug;
}

// Serializable Quaternion
[Serializable]
public struct SQuaternion
{
    public float x;
    public float y;
    public float z;
    public float w;

    public SQuaternion(float rX, float rY, float rZ, float rW)
    {
        x = rX;
        y = rY;
        z = rZ;
        w = rW;
    }

    public override string ToString()
    {
        return String.Format("[{0}, {1}, {2}, {3}]", x, y, z, w);
    }

    public static implicit operator Quaternion(SQuaternion rValue)
    {
        return new Quaternion(rValue.x, rValue.y, rValue.z, rValue.w);
    }

    public static implicit operator SQuaternion(Quaternion rValue)
    {
        return new SQuaternion(rValue.x, rValue.y, rValue.z, rValue.w);
    }
}
