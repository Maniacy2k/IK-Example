using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ik_Test : MonoBehaviour {

    public UnityEngine.UI.Text[] lbl_jointAngles;
    public UnityEngine.UI.Text lbl_Cycles;

    public GameObject[] joints;
    private GameObject[] jointRots;
    public GameObject target;

    [Tooltip("EPS = Epsilon, how far end effector position will be from target position")]
    public float EPS = 0.5f;
    [Tooltip("Step value = How many angle change for each joint per iteration")]
    public float step = 0.015f;

    private float[] angles;
    private int count = 0;
    [Tooltip("IK iteration cycle maximum!")]
    public int countMax = 1000;
    private bool startJT_Method_Flag = false;


    #region Unity_Methods

    void Start()
    {
        jointRots = new GameObject[joints.Length-1];
        for (int i = 0; i < jointRots.Length; i++)
        {
            GameObject tmp = new GameObject(joints[i + 1].name + "_Rot");
            tmp.transform.position = joints[i+1].transform.position;
            tmp.transform.parent = joints[i].transform;
            jointRots[i] = tmp;            
        }        
    }

    private void Update()
    {
        if (startJT_Method_Flag) {
            iterate_IK();
        }

        if (Input.GetKeyDown(KeyCode.F))
        {
            start_IK();
        }

    }

    public void btn_StartIK() {
        start_IK();
    }


    #endregion


    #region IK_Method

    private void start_IK()
    {
        count = 0;
        startJT_Method_Flag = true;

        float angleA = calculateAngle(Vector3.up, joints[1].transform.position, joints[0].transform.position);
        float angleB = calculateAngle(Vector3.up, joints[2].transform.position, joints[1].transform.position);
        float angleC = calculateAngle(Vector3.up, joints[3].transform.position, joints[2].transform.position);
        angles = new float[] { angleA, angleB, angleC };
    }

    private void iterate_IK()
    {
        if (Mathf.Abs(Vector3.Distance(joints[joints.Length - 1].transform.position, target.transform.position)) > EPS)
        {
            JacobianIK();
            lbl_Cycles.text = string.Format("Cycle: {0}", count.ToString("0###"));
        }
        else
        {
            Debug.Log("Cycle Count: " + count.ToString());
            startJT_Method_Flag = false;
            lbl_Cycles.text = string.Format("Target reached, cycle Count:{0}!", count.ToString("0###"));
        }
        if (count >= countMax)
        {
            Debug.Log("Hit Cycle Count: " + count.ToString());
            lbl_Cycles.text = string.Format("Cycle max {0} reached, target reached!", countMax.ToString());
            startJT_Method_Flag = false;
        }
    }
    
    #region Jacobian_Transpose

    private void JacobianIK()
    {

        float[] dO = new float[angles.Length];
        float[] angleDiff = new float[angles.Length];
        dO = GetDeltaOrientation();
        for (int i = 0; i < dO.Length; i++) {
            angles[i] += dO[i] * step;
            angleDiff[i] = dO[i] * step;
        }
        
        // update angles
        rotateLinks(angleDiff);

        Tools.displayDebug("Angles: " + angles.ToString());
        count++;
    }

    private float[] GetDeltaOrientation()
    {
        float[,] Jt = GetJacobianTranspose();

        Vector3 V = (target.transform.position- joints[joints.Length - 1].transform.position);

        //dO = Jt * V;
        float[,] dO = Tools.M_Multiply(Jt, new float[,] { { V.x }, { V.y }, { V.z  } });
        return new float[] { dO[0, 0], dO[1, 0], dO[2, 0] };
    }

    private float[,] GetJacobianTranspose() {

        Vector3 J_A = Vector3.Cross(joints[0].transform.forward, (joints[joints.Length - 1].transform.position - joints[0].transform.position));
        Vector3 J_B = Vector3.Cross(joints[1].transform.forward, (joints[joints.Length - 1].transform.position - joints[1].transform.position));
        Vector3 J_C = Vector3.Cross(joints[2].transform.forward, (joints[joints.Length - 1].transform.position - joints[2].transform.position));

        float[,] matrix = new float[3, 3];

        matrix = Tools.M_Populate(matrix, new Vector3[] { J_A, J_B, J_C });

        return Tools.M_Transpose(matrix);
    }

    #endregion

    #region IK_Chain

    private float calculateAngle(Vector3 axis, Vector3 pos1, Vector3 pos2)
    {
        float value = 0f;

        value = Vector3.Angle(axis, (pos1 - pos2).normalized);
        Vector3 cross = Vector3.Cross(axis, (pos1 - pos2).normalized);
        if (cross.z < 0)
            value = -value;

        return value;
    }

    private void rotateLinks(float[] angleDiff)
    {
        float[] displayAngles = new float[angleDiff.Length];

        for (int i = 0; i < joints.Length - 1; i++)
        {            
            Vector3 upDir = joints[i].transform.right;

            Vector3 crossAxis = Vector3.Cross(upDir, (joints[i + 1].transform.position - joints[i].transform.position).normalized);
            float currAngle = calculateAngle(Vector3.up, joints[i + 1].transform.position, joints[i].transform.position);
            float newAngle = angleDiff[i];
            displayAngles[i] = angleDiff[i] + currAngle;

            if (newAngle != 0)
                joints[i].transform.RotateAround(joints[i].transform.position, crossAxis, newAngle);

            if (i < joints.Length - 2)
                updateLinkPos(i, joints[i].transform.position, crossAxis, angleDiff[i]);
            if (i >= joints.Length - 2) // end effector
                joints[i + 1].transform.position = jointRots[i].transform.position;

            //Debug.Log("joint " + (i + 1).ToString() + ": New angle Value: " + angleDiff[i].ToString());
        }

        display_JointAngles(angles, displayAngles);
    }
        
    private void updateLinkPos(int p, Vector3 rotPos, Vector3 cross, float angle)
    {
        if (p >= joints.Length - 2)
            return;

        for (int i = p; i < jointRots.Length; i++)
            joints[i + 1].transform.position = jointRots[i].transform.position;

        return;

    }

    private void display_JointAngles(float[] angles, float[] actualAngles)
    {
        string formatValues = "0##.#0";
        lbl_jointAngles[0].text = string.Format("JointA JK: {0}, Actual: {1}", angles[0].ToString(formatValues), actualAngles[0].ToString(formatValues));
        lbl_jointAngles[1].text = string.Format("JointB JK: {0}, Actual: {1}", angles[1].ToString(formatValues), actualAngles[1].ToString(formatValues));
        lbl_jointAngles[2].text = string.Format("JointC JK: {0}, Actual: {1}", angles[2].ToString(formatValues), actualAngles[2].ToString(formatValues));
    }

    private void resetJoints()
    {
        for (int i = 0; i < joints.Length - 1; i++)
        {
            joints[i].transform.rotation = new Quaternion(0f, 0f, 0f, joints[i].transform.rotation.w);

            if (i > 0)
                joints[i].transform.position = jointRots[i - 1].transform.position;
            if (i >= joints.Length - 2) // end effector
                joints[i + 1].transform.position = jointRots[i].transform.position;
        }

    }

    #endregion

    #endregion

}
