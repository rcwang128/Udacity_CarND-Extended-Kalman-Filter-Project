# **Extended Kalman Filter** 

## Harry Wang
---

**Extended Kalman FIlter Project**

The goals / steps of this project are the following:
* Implement the extended Kalman filter in C++
* Use provided lidar and radar measurements data to detecting a bicycle that travels around the vehicle
* Track object's movement with markers and calculate RMSE from given ground truth data
* Compile the code without errors
* Set up the simulator with uWebSocketIO to visualize object's trajectory



The full project can be found at
https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project

---
### Procedures

Kalman Filter represents the distributions by Gaussians and iterates on two main cycles: Prediction and Measurement update.

![kalmanfilter.png](https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project/blob/master/pic/kalmanfilter.png?raw=true)



During prediction cycle, we predict the object's new location using current position and velocity. Uncertainty increases since object may not maintain the exact same velocity or direction. Process noise should also be taken into considerations.

During measurement update cycle, we calculate current position and (or) velocity. And then update new state with considerations of measurement noise.

Unlike lidar measurement data that is consisted of linear position and velocity functions, radar measures object's range, bearing and radial velocity, which are not cross-correlated. The mapping from measurement to state vector is non-linear. Therefore for radar, we use extended kalman filter to linearize the measurement function before updating new state.

![EKF.png](https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project/blob/master/pic/EKF.png?raw=true)

An overview of the Kalman filter implementation for this project.

![EKF_diagram.png](https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project/blob/master/pic/EKF_diagram.png?raw=true)



### File Structure

#### Files in the src Folder

- `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE

- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function

- `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar

- `tools.cpp`- function to calculate RMSE and the Jacobian matrix

  

### Project Code Highlights

#### FusionEKF.cpp

Initialize the Kalman filter position vector with the first sensor measurements.

```c++
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      float ro, theta, ro_dot;
      ro = measurement_pack.raw_measurements_[0];
      theta = measurement_pack.raw_measurements_[1];
      ro_dot = measurement_pack.raw_measurements_[2];

      // Convert to cartesian coordinates and update px,py,vx,vy
      px = ro * cos(theta);
      py = ro * sin(theta);
      vx = ro_dot * cos(theta);
      vy = ro_dot * sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state. vx = vy = 0 for initialization
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      vx = 0;
      vy = 0;
    }
    ekf_.x_ << px, py, vx, vy;
```

Update the state transition matrix F and process noise covariance matrix Q. And call the predict function.

```c++
  // initialize transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
```

```c++
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // Update state transition matrix with delta t
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Set the process covariance matrix Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_4*noise_ax/4, 0, dt_3*noise_ax/2, 0,
            0, dt_4*noise_ay/4, 0, dt_3*noise_ay/2,
            dt_3*noise_ax/2, 0, dt_2*noise_ax, 0,
            0, dt_3*noise_ay/2, 0, dt_2*noise_ay;

  ekf_.Predict();
```

Call the update step for either the lidar or radar sensor measurement. 

```c++
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
}
```



#### kalman_filter.cpp

Complete kalman filter predict function.

```c++
void KalmanFilter::Predict() {
  /**
   * predict the state
   */ 
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
```

Complete kalman filter update function for lidar measurement.

```c++
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
```

Complete extended kalman filter update function for radar measurement.

```c++
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */ 
  VectorXd z_pred = CartesianToPolar(x_);
  VectorXd y = z - z_pred;

  // normalize the angle
  while(y(1) > M_PI || y(1) < -M_PI) {
    if (y(1) > M_PI) {
      y(1) -= M_PI;
    } 
    else {
      y(1) += M_PI;
    }
  }

  // same as Kalman filter update
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
```



#### tools.cpp

Complete RMSE from ground truth values.

```c++
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  /* Check the validity of the following inputs:
  * the estimation vector size should not be zero;
  * the estimation vector size should equal ground truth vector size.
  */
  if (estimations.size() != ground_truth.size()) {
    cout << "Invalid estimation or ground_truth data!" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}
```

Complete Jacobian function.

```c++
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
```



### Results

#### Object tracking with Lidar and Radar measurement.

Red marker is Lidar measurement while blue marker is Radar measurement.

With use of two types of sensor measurements, my kalman filter algorithm is able to compute estimated location (green marker) of the vehicle. And the error is displayed in RMSE format after comparing with ground truth trajectory.

![tracking_zoomin](https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project/blob/master/pic/zoomin.png?raw=true)

Overview of Dataset 1 trajectory.

![dataset1.png](https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project/blob/master/pic/dataset1.png?raw=true)



Overview of Dataset 2 trajectory.

![dataset2.png](https://github.com/rcwang128/Udacity_CarND-Extended-Kalman-Filter-Project/blob/master/pic/dataset2.png?raw=true)