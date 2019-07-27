using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class analytical_IK : MonoBehaviour
{
    public GameObject[] joints;
    public GameObject target;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.G))
        {
            calc_AnaIK();
        }
    }

    private void calc_AnaIK()
    {
        resetJoints();

        Vector3 targetPos = target.transform.position;
        float d1 = joints[1].transform.position.magnitude;
        float d2 = joints[2].transform.position.magnitude - d1;
        
        if (target.transform.position.magnitude > (d1 + d2))
        {
            Debug.Log("To far");
            return;
        }
        if (target.transform.position.magnitude < (d1 - d2))
        {
            Debug.Log("To close");
            return;
        }
        
        float angle2 = 0f;
        float angle1 = 0f;
        float targetDistSqr = targetPos.x * targetPos.x + targetPos.y * targetPos.y;
        float cosAngle2_denom = 2 * d1 * d2;

        float cosAngle2 = (targetDistSqr - (d1 * d1) - (d2 * d2)) / (cosAngle2_denom);


        // compute a new value for angle2
        angle2 = Mathf.Acos(cosAngle2) * 180 / Mathf.PI;

        // compute the sine of our angle
        float sinAngle2 = Mathf.Sin(angle2 * Mathf.PI / 180);

        //===
        // Compute the value of angle1 based on the sine and cosine of angle2
        float triAdjacent = d1 + d2 * cosAngle2;
        float triOpposite = d2 * sinAngle2;

        float tanY = targetPos.y * triAdjacent - targetPos.x * triOpposite;
        float tanX = targetPos.x * triAdjacent + targetPos.y * triOpposite;

        // Note that it is safe to call Atan2(0,0) which will happen if targetX and
        // targetY are zero
        float tmpAtan2 = Mathf.Atan2(tanY, tanX);
        angle1 = tmpAtan2 * 180 / Mathf.PI;
        Debug.Log(string.Format("Angle1:{0}, Angle2:{1}", angle1.ToString(), angle2.ToString()));

        //float j1Angle = Vector3.Angle(Vector3.up, (joints[1].transform.position - joints[0].transform.position).normalized);
        //float j2Angle = Vector3.Angle(Vector3.up, (joints[2].transform.position - joints[1].transform.position).normalized);
        //Debug.Log(string.Format("J1 Angle:{0}, J2 Angle:{1}", j1Angle.ToString(), j2Angle.ToString()));

        Vector3 cross1 = Vector3.Cross(Vector3.right, (joints[1].transform.position - joints[0].transform.position).normalized);
        Vector3 cross2 = Vector3.Cross(Vector3.right, (joints[2].transform.position - joints[1].transform.position).normalized);

        joints[0].transform.RotateAround(joints[0].transform.position, cross1, -(90-angle1));
        joints[1].transform.RotateAround(joints[1].transform.position, cross2, angle2);
    }

    private void resetJoints()
    {
        for (int i = 0; i < joints.Length; i++)        
            joints[i].transform.rotation = new Quaternion(0f, 0f, 0f, joints[i].transform.rotation.w);
        
    }
}
