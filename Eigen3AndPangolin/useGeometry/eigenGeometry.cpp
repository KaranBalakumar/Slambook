#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// This program demonstrates the usage of Eigen's geometry module.

int main(int argc, char **argv) {

  // Eigen/Geometry The module provides representations for various rotations and translations.
  // 3D rotation matrices can be directly represented using Matrix3d or Matrix3f.
  Matrix3d rotation_matrix = Matrix3d::Identity();
  

// Rotation vectors use AngleAxis. It is not directly a Matrix at its core, but operations 
// can be treated as matrices (since operators are overloaded).
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1)); // Rotate 45 degrees around the Z axis.
  cout.precision(3);
  cout << "rotation matrix =\n" << rotation_vector.matrix() << endl; // Convert to matrix using matrix() function.
  // It can also be directly assigned.
  rotation_matrix = rotation_vector.toRotationMatrix();
  // Coordinate transformations can be performed using AngleAxis.
  Vector3d v(1, 0, 0);
  Vector3d v_rotated = rotation_vector * v;
  cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
  // Alternatively, rotation matrices can be used.
  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  // Euler angles: Rotation matrices can be directly converted to Euler angles.
  Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order, i.e., yaw-pitch-roll order.
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // Euclidean transformation matrices use Eigen::Isometry.
  Isometry3d T = Isometry3d::Identity(); // Although named 3D, it is essentially a 4x4 matrix.              
  T.rotate(rotation_vector); // Rotate according to the rotation_vector.
  T.pretranslate(Vector3d(1, 3, 4)); // Set the translation vector to (1, 3, 4).
  cout << "Transform matrix = \n" << T.matrix() << endl;

  // Perform coordinate transformation using the transformation matrix.
  Vector3d v_transformed = T * v; // Equivalent to R*v + t.
  cout << "v tranformed = " << v_transformed.transpose() << endl;

  // For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d respectively.

  // Quaternions
  // AngleAxis can be directly assigned to quaternions, and vice versa.

  Quaterniond q = Quaterniond(rotation_vector);
  cout << "quaternion from rotation vector = " << q.coeffs().transpose()
       << endl;   // Please note that the order of coeffs is (x, y, z, w), where w is the real part and the first three are the imaginary parts.
  // Rotation matrices can also be assigned to it.
  q = Quaterniond(rotation_matrix);
  cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
  
  // Rotate a vector using a quaternion, simply using overloaded multiplication.
  v_rotated = q * v; // Note that mathematically it is qvq^{-1}.
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  // If expressed using conventional vector multiplication, it should be calculated as follows:
  // v_rotated = q * v * q.conjugate();
  cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

  return 0;
}
