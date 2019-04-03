#include <i_see_pee/i_see_pee.hpp>
#include <i_see_pee/macros.hpp>
#include <i_see_pee/iterator.hpp>
#include <i_see_pee/debug.hpp>
#include <i_see_pee/utils.hpp>

#include <tf2/utils.h>

namespace i_see_pee {

namespace internal {

transform_t to_transform(const transform_t__ &_tf) noexcept {
  // cast is necessary since the internal floating point format of
  // i_see_pee is float and the from tf2 double
  const auto x = static_cast<float>(_tf.transform.translation.x);
  const auto y = static_cast<float>(_tf.transform.translation.y);
  const auto yaw = static_cast<float>(tf2::getYaw(_tf.transform.rotation));
  return Eigen::Translation2f(x, y) * Eigen::Rotation2Df(yaw);
}

transform_t to_transform(const pose_t__ & _p) noexcept{
  // cast is necessary since the internal floating point format of
  // i_see_pee is float and the from tf2 double
  const auto x = static_cast<float>(_p.position.x);
  const auto y = static_cast<float>(_p.position.y);
  const auto yaw = static_cast<float>(tf2::getYaw(_p.orientation));
  return Eigen::Translation2f(x, y) * Eigen::Rotation2Df(yaw);
}

transform_t__ to_transform(const transform_t &_t) noexcept {
  // pull eigen namespace up
  using namespace Eigen;

  // get easy access to internals
  // cast is necessary since the internal floating point format of
  // i_see_pee is float and the from tf2 double
  const Vector2d t = _t.translation().cast<double>();
  const Matrix2d r = _t.rotation().cast<double>();

  // get the angle from the rotation matrix
  const auto angle = std::atan2(r(1, 0), r(0, 0));

  // convert the SO2 of transform_t to SO3 of transform_t__
  transform_t__ out;
  out.transform.translation.x = t(0);
  out.transform.translation.y = t(1);
  out.transform.translation.z = 0;

  Quaternion<double> q;
  q = AngleAxis<double>(angle, Vector3d::UnitZ());

  out.transform.rotation.x = q.x();
  out.transform.rotation.y = q.y();
  out.transform.rotation.z = q.z();
  out.transform.rotation.w = q.w();

  return out;
}

transform_t__ to_transform(const transform_t &_t,
                           const stamp_t& _stamp,
                           const std::string &_frame,
                           const std::string &_child) noexcept {
  auto out = to_transform(_t);
  out.header.stamp = to_stamp(_stamp);
  out.header.frame_id = _frame;
  out.child_frame_id = _child;

  return out;
}

} // namespace internal

namespace map{

map_data::map_data(const nav_msgs::MapMetaData &m) noexcept:
        resolution(std::abs(m.resolution)),
        origin(m.origin.position.x, m.origin.position.y),
        size(m.width, m.height) {}

map_data::map_data(const coordinate_t &_size,
                   float _resolution,
                   const position_t &_origin) noexcept:
        size(_size), resolution(std::abs(_resolution)), origin(_origin) {}

converter::converter(const map_data &_p) noexcept : p_(_p) {}
converter::converter(const nav_msgs::MapMetaData &m) noexcept : p_(m) {}

coordinate_t converter::to_coordinate(const position_t &_p) const {
  // get the relative position
  const position_t relative = _p - p_.origin;
  if (relative.minCoeff() < 0) {
    throw std::out_of_range("position out of bounds");
  }

  // cast the relative position to coordinate
  const coordinate_t out = (relative / p_.resolution).cast<size_t>();
  if (!valid_coordinate(out)) {
    throw std::out_of_range("position out of bounds");
  }
  return out;
}

coordinate_t converter::to_coordinate(index_t i) const {
  size_t col = i % p_.size(0);
  size_t row = i / p_.size(0);
  const coordinate_t out(col, row);
  if (!valid_coordinate(out)) {
    throw std::out_of_range("index out of bounds");
  }
  return out;
}

index_t converter::to_index(const position_t &p) const {
  // can throw std::runtime_error if the position is invalid
  const coordinate_t c = to_coordinate(p);
  return to_index(c);
}

index_t converter::to_index(const coordinate_t &c) const {
  if (!valid_coordinate(c)) {
    throw std::out_of_range("coordinate out of bounds");
  }
  return c(0) + c(1) * p_.size(0);
}

position_t converter::to_position(index_t _i) const {
  if (!valid(_i)) {
    throw std::out_of_range("invalid index");
  }
  // cannot throw anymore
  const coordinate_t cor = to_coordinate(_i);
  return to_position(cor);
}

position_t converter::to_position(const coordinate_t &_c) const {
  return _c.cast<float>() * p_.resolution +
         p_.origin + position_t::Ones() * p_.resolution / 2.;
}

knn_t::knn_t(const nav_msgs::MapMetaData &_grid, size_t _k) noexcept:
        converter_(_grid), k_(_k) {}

knn_t::knn_t(const map_data &_parameter, size_t _k) noexcept:
        converter_(_parameter), k_(_k) {}

constexpr size_t knn_parameter::k_max;
constexpr size_t knn_parameter::k_min;
constexpr size_t knn_parameter::k_def;

constexpr size_t knn_parameter::radius_max;
constexpr size_t knn_parameter::radius_min;
constexpr size_t knn_parameter::radius_def;

knn_parameter::knn_parameter(size_t _k, size_t _radius) noexcept :
        k_(cast_to_range(_k, k_min, k_max)),
        radius_(cast_to_range(_radius, radius_min, radius_max)) {}

knn_parameter::knn_parameter(ros::NodeHandle &_nh) {
  ros::NodeHandle pnh(_nh, "map");

  // ros::NodeHandle does not support unsigned int, hence we must take a way
  // over int and then cast the raw value to unsigned
  const auto k_raw = pnh.param("knn", static_cast<int>(k_def));
  k_ = cast_to_range(static_cast<size_t>(k_raw), k_min, k_max);

  const auto radius_raw = pnh.param("radius", static_cast<int>(radius_def));
  radius_ = cast_to_range(static_cast<size_t>(radius_raw), radius_min, radius_max);

  I_SEE_PEE_INFO(*this);
}

std::ostream &operator<<(std::ostream &_os, const knn_parameter &_p) noexcept {
  _os << "<<k " << _p.k_ << ", radius " << _p.radius_ << ">>";
  return _os;
}

is_reachable::is_reachable(const nav_msgs::OccupancyGrid &_grid) noexcept :
        grid_(_grid), converter_(_grid.info) {}

bool is_reachable::operator()(index_t _center) noexcept {
  // pull the submap iterator and its defines into namespace
  using iterator_t = submap_iterator<long int>;
  using data_t = typename iterator_t::data_t;
  using bound_t = typename iterator_t::bound_t;

  const bound_t moore_neighbors(3, 3);

  // check if the center is valid
  if (!converter_.valid(_center)) {
    return false;
  }

  // prepare for the iteration
  const coordinate_t center_c = converter_.to_coordinate(_center);
  const data_t start_c = center_c.cast<long int>() - data_t::Ones();
  static const bound_t moore_size(3, 3);

  // check the moore-neighborhood around the center
  for (iterator_t ii(start_c, moore_size); !ii.past_end(); ++ii) {
    // check if the coordinate is valid
    const coordinate_t current_c = ii->cast<size_t>();
    if (!converter_.valid_coordinate(current_c)) {
      continue;
    }
    // cannot throw at this point anymore
    const index_t current_i = converter_.to_index(current_c);
    if (!is_occupied(grid_.data[current_i])) {
      return true;
    }
  }
  return false;
}

knn_ptr_t interpret(const nav_msgs::OccupancyGrid &_grid,
                    const knn_parameter &_p) noexcept {
  // pull definitions into namespace
  using circle_iterator_t = circle_iterator<long int>;
  using data_t = typename circle_iterator_t::data_t;

  // setup the output structure
  knn_ptr_t out = knn_ptr_t(new knn_t(_grid.info, _p.k_));
  const auto &c = out->converter_;

  is_reachable reachable(_grid);

  // iterate over the entire grid
  for (size_t index = 0; index != _grid.data.size(); ++index) {
    // pick only the occupied cells
    if (!is_occupied(_grid.data[index])) {
      continue;
    }

    // check if the cell is reachable
    if (!reachable(index)) {
      continue;
    }

    // rotate around the occupied cells
    const data_t center = c.to_coordinate(index).cast<long int>();
    const position_t p_occ = c.to_position(index);
    for (circle_iterator_t ci(center, _p.radius_); !ci.past_end(); ++ci) {
      const coordinate_t c_nb = ci->cast<size_t>();
      // skip the cells of the grid
      if (!c.valid_coordinate(c_nb)) {
        continue;
      }

      // save the index to the hash_map.
      // conversion to_index cannot throw, since we checked that the
      // coordinate is valid.
      const index_t i_nb = c.to_index(c_nb);

      // if the cell has no knn's yet reserve for it
      if (out->knn_.find(i_nb) == out->knn_.end()) {
        out->knn_[i_nb].reserve(_p.k_);
      }

      const position_t p_nb = c.to_position(i_nb);
      // maybe_insert will insert if the new value is nearer than the furthest
      // point or if the vector has not reached its full capacity
      maybe_insert(out->knn_[i_nb], p_occ,
              [&p_nb](const position_t& _l, const position_t& _r)
              {return (p_nb - _l).norm() < (p_nb - _r).norm();});
    }
  }
  return std::move(out);
}

interface::interface(ros::NodeHandle &_nh) : p_(_nh), knn_(nullptr) {
  ros::NodeHandle pnh(_nh, "map");

  std::string topic = pnh.param("topic", std::string{"/map"});
  sub_ = pnh.subscribe(topic, 1, &interface::callback, this);
}

bool interface::move_data(knn_ptr_t &_knn) noexcept {
  if (knn_) {
    // will reset the knn_ to nullptr as defined in
    // https://en.cppreference.com/w/cpp/memory/unique_ptr
    _knn = std::move(knn_);
    return true;
  }
  return false;
}

void interface::callback(const nav_msgs::OccupancyGridConstPtr &_msg) noexcept {
  // before using the raw data, verify its integrity
  if (!is_valid(*_msg)) {
    I_SEE_PEE_WARN("map info does not match grid-size");
    return;
  }

  I_SEE_PEE_INFO("stating...");
  knn_ = interpret(*_msg, p_);
  I_SEE_PEE_INFO("done.");
}

} // namespace map

namespace scan {

parameter::parameter(const sensor_msgs::LaserScan &_scan) noexcept :
        size(_scan.ranges.size()),
        increment(_scan.angle_increment),
        angle_min(_scan.angle_min),
        angle_max(_scan.angle_max) {
  // check the integrity of the scan data: the scan rays are evenly distributed
  // between [angle_min, angle_max] with both ends populated
  const auto raw_size = (angle_max - angle_min) / increment + 1.f;
  const auto imp_size = static_cast<size_t>(std::round(std::abs(raw_size)));
  if (imp_size != size) {
    I_SEE_PEE_WARN("implied size " << imp_size
                                   << " differs from real size: " << size);
  }
}

bool parameter::operator==(const parameter &_other) const noexcept {
  return size == _other.size &&
         increment == _other.increment &&
         angle_min == _other.angle_min &&
         angle_max == _other.angle_max;
}

bool parameter::operator!=(const parameter &_other) const noexcept {
  return !(*this == _other);
}

scan_ptr_t cache(const parameter &_p) noexcept {
  // allocate the data for the cache
  scan_ptr_t out = scan_ptr_t(new scan_t(2, _p.size));
  float angle = _p.angle_min;
  // cache the transformation of the sensor_msgs::LaserScan to scan::scan_t
  for (size_t ii = 0; ii < _p.size; ++ii) {
    out->col(ii) << std::cos(angle), std::sin(angle);
    angle += _p.increment;
  }
  return std::move(out);
}

interface_::interface_(controller_base_ *_controller) :
        controller_(_controller), p_(nullptr), cache_(nullptr) {}

void interface_::callback(const sensor_msgs::LaserScanConstPtr &_msg) {
  parameter_ptr_t p_new = parameter_ptr_t(new parameter(*_msg));
  // update the cache if the scan parameters have changed
  // todo change to hash map to support multiple topics
  if (!p_ || *p_ != *p_new) {
    p_ = std::move(p_new);
    cache_ = cache(*p_);
  }

  // avoid accessing the data when the vector is empty, since this is
  // not defined by the c++ standard:
  // https://en.cppreference.com/w/cpp/container/vector/data
  if (_msg->ranges.empty()) {
    return;
  }

  // use Eigen::Map to allow Matrix operation on ranges
  scan_view_t ranges(_msg->ranges.data(), _msg->ranges.size());
  // convert the sensor data to internal scan_t
  scan_t scan(2, p_->size);
  // this operation is optimal if we use Eigen::RowMajor storage order
  scan.row(0) = cache_->row(0).array() * ranges.array();
  scan.row(1) = cache_->row(1).array() * ranges.array();

  if (controller_) {
    const auto stamp = internal::to_stamp(_msg->header.stamp);
    controller_->update(scan, stamp, _msg->header.frame_id);
  }
}

interface::interface(ros::NodeHandle &_nh, controller_base_ *_controller) :
        interface_(_controller) {
  ros::NodeHandle pnh(_nh, "scan");
  const std::string topic = pnh.param("topic", std::string{"/scan"});
  sub_ = _nh.subscribe(topic,
                       1,
                       &interface::callback,
                       dynamic_cast<interface_ *>(this));
}

} // namespace scan

namespace odom {

interface_tf::interface_tf() : tf_listener_(buffer_) {}

transform_t interface_tf::get(const std::string &_from, const std::string &_to) {
  // will throw if the tf::lookupTransform fails
  const auto out = buffer_.lookupTransform(_from, _to, ros::Time(0));
  return internal::to_transform(out);
}

void interface_tf::set(const transform_t &_tf,
                       const stamp_t &_stamp,
                       const std::string &_from,
                       const std::string &_to) noexcept {
  const auto out = internal::to_transform(_tf, _stamp, _from, _to);
  tf_broadcaster_.sendTransform(out);
}

constexpr char interface_rviz::topic[];

interface_rviz::interface_rviz(ros::NodeHandle &_nh,
                               transform_t &_map_to_base) noexcept
        : map_to_base_(_map_to_base) {
  ros::NodeHandle pnh(_nh, "odom");
  map_frame_ = pnh.param("map_frame", std::string{"map"});
  sub_ = _nh.subscribe(std::string{topic}, 1, &interface_rviz::callback, this);
}

void interface_rviz::callback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr &_msg) noexcept {
  // check if the frame is valid (must be "map")
  if (_msg->header.frame_id != map_frame_) {
    I_SEE_PEE_WARN("the frame must be " << map_frame_);
    return;
  }

  // check if rotation is valid
  if (!internal::is_valid(_msg->pose.pose.orientation)) {
    I_SEE_PEE_WARN("ill configured quaternion");
    return;
  }

  I_SEE_PEE_INFO("updating initial pose");
  map_to_base_ = internal::to_transform(_msg->pose.pose);
}

frame_handler::frame_handler(ros::NodeHandle &_nh) :
        rviz_(_nh, map_to_base_),
        base_to_sensor_(transform_t::Identity()),
        map_to_base_(transform_t::Identity()),
        odom_to_base_(transform_t::Identity()) {
  ros::NodeHandle pnh(_nh, "odom");

  map_frame_ = pnh.param("map_frame", std::string{"map"});
  base_frame_ = pnh.param("base_frame", std::string{"base_link"});
  odom_frame_ = pnh.param("odom_frame", std::string{"odom"});
}

void frame_handler::handle_map_to_base() {
  // will throw if lookup transform fails
  const transform_t odom_to_new = tf_.get(odom_frame_, base_frame_);

  // prepare new data
  // todo set flag if no motion was registered
  const transform_t base_to_new = odom_to_base_.inverse() * odom_to_new;
  const transform_t map_to_new = map_to_base_ * base_to_new;

  // update the buffers
  odom_to_base_ = odom_to_new;
  map_to_base_ = map_to_new;
}

void frame_handler::handle_base_to_sensor(const std::string &_scan_frame) {
  // will throw if lookup transform fails
  base_to_sensor_ = tf_.get(base_frame_, _scan_frame);
}

transform_t frame_handler::get_map_to_base() noexcept {
  try {
    handle_map_to_base();
  }
  catch (std::runtime_error &ex) {
    I_SEE_PEE_WARN(ex.what());
  }

  return map_to_base_;
}

transform_t frame_handler::get_map_to_odom() const noexcept {
  return map_to_base_ * odom_to_base_.inverse();
}

transform_t
frame_handler::get_map_to_odom(const transform_t &_map_to_base) noexcept {
  map_to_base_ = _map_to_base;
  return get_map_to_odom();
}

transform_t
frame_handler::get_base_to_sensor(const std::string &_scan_frame) noexcept {
  try {
    handle_base_to_sensor(_scan_frame);
  }
  catch (tf2::TransformException &ex) {
    I_SEE_PEE_WARN(ex.what());
  }

  return base_to_sensor_;
}

void frame_handler::set_map_to_odom(const transform_t &_tf,
                                    const stamp_t &_stamp) noexcept {
  tf_.set(_tf, _stamp, map_frame_, odom_frame_);
}

} // namespace odom

namespace icp {

matches::matches(const scan::scan_t &_sensor, const scan::scan_t &_map) noexcept:
        sensor_(_sensor), map_(_map) {}

matches::matches(const scan::scan_t &_scan, const map::knn_t &_knn) noexcept :
        sensor_(_scan.rows(), _scan.cols() * _knn.k_),
        map_(_scan.rows(), _scan.cols() * _knn.k_) {
  size_t jj = 0;
  // iterate over the input scan
  for (size_t cc = 0; cc < _scan.cols(); ++cc) {
    const map::position_t pp = _scan.col(cc).transpose();
    if (!_knn.converter_.valid_position(pp)) {
      continue;
    }

    // cannot throw since we checked that the position is valid
    const auto ii = _knn.converter_.to_index(pp);

    // discard sensor readings which have no known knn's
    if (_knn.knn_.find(ii) == _knn.knn_.end()) {
      continue;
    }

    // unordered_map::at can throw an out_of_range; but since we have checked
    // in advance, it's quasi exception save.
    // https://en.cppreference.com/w/cpp/container/unordered_map/at
    const auto &matches = _knn.knn_.at(ii);

    // iterate over known knn's of the reading
    const auto n_cols = matches.size();
    for (size_t nn = 0; nn < matches.size(); ++jj, ++nn) {
      map_.col(jj) = matches[nn];
      sensor_.col(jj) = _scan.col(cc);
    }
  }

  // resize the generated point clouds
  map_.conservativeResize(Eigen::NoChange, jj);
  sensor_.conservativeResize(Eigen::NoChange, jj);
}

weight_t get_weights(const matches &_data) {
  // assert that the input data is well conditioned
  assert(_data.map_.cols() == _data.sensor_.cols());
  const auto cols = _data.map_.cols();
  weight_t out(cols);

  // iterate over match pairs
  for (size_t cc = 0; cc < cols; ++cc) {
    const auto unused = (_data.map_.col(cc) - _data.sensor_.col(cc)).norm();
    out(cc) = 1;
  }

  return std::move(out);
}

transform_t point_to_map(matches&& _data, weight_t&& _w){
  // this code is a shameless ripoff of the ethz libpointmatcher point_to_point.
  // https://github.com/ethz-asl/libpointmatcher
  // http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
  using vector_t = Eigen::Vector2f;
  using matrix_t = Eigen::Matrix2Xf;

  const auto w_sum = _w.sum();
  // get the mean of sensor and map
  const vector_t mean_sensor = (_data.sensor_ * _w) / w_sum;
  const vector_t mean_map = (_data.map_ * _w) / w_sum;

  // remove the mean from sensor and map
  _data.sensor_.colwise() -= mean_sensor;
  _data.map_.colwise() -= mean_map;

  // singular value decomposition
  const matrix_t m(_data.map_ * _w.asDiagonal() * _data.sensor_.transpose());
  const Eigen::JacobiSVD<matrix_t> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // get the rotation
  matrix_t rotation(svd.matrixU() * svd.matrixV().transpose());
  if (rotation.determinant() < 0.){
    matrix_t tmpV = svd.matrixV().transpose();
    tmpV.row(0) *= -1.;
    rotation = svd.matrixU() * tmpV;
  }

  // get the translation
  const vector_t translation(mean_map - rotation * mean_sensor);

  // assembly the result
  transform_t result;
  result.linear() = rotation;
  result.translation() = translation;
  return result;
}

constexpr float conversion_checker::t_norm;
constexpr float conversion_checker::r_norm;

conversion_checker::conversion_checker(float _t_norm, float _r_norm) noexcept :
        t_norm_(std::abs(_t_norm)), r_norm_(std::abs(_r_norm)) {}

conversion_checker::conversion_checker(ros::NodeHandle &_nh) {
  ros::NodeHandle pnh(_nh, "icp");

  t_norm_ = std::max(pnh.param("t_norm", t_norm), t_norm);
  r_norm_ = std::max(pnh.param("r_norm", r_norm), r_norm);
}

bool conversion_checker::operator()(const transform_t &_in) noexcept {
  const auto t_curr = _in.translation().norm();
  const auto angle = std::atan2(_in.linear()(1, 0), _in.linear()(0, 0));
  const auto r_curr = std::abs(angle);

  return t_norm_ > t_curr && r_norm_ > r_curr;
}

constexpr size_t sampler::stride_def;
constexpr size_t sampler::stride_min;
constexpr size_t sampler::stride_max;

sampler::sampler(size_t _stride) noexcept :
        stride_(cast_to_range(_stride, stride_min, stride_max)),
        gen_(rd_()),
        dist_(distribution_t::param_type(0, stride_ - 1)) {}

sampler::sampler(ros::NodeHandle &_nh) : gen_(rd_()) {
  ros::NodeHandle pnh(_nh, "icp");

  // use workaround through int, since ros::NodeHandle cannot handle size_t
  const auto raw = pnh.param("stride", static_cast<int>(stride_def));
  stride_ = cast_to_range(static_cast<decltype(stride_def)>(raw),
                          stride_min,
                          stride_max);
  // adjust the parameter of the uniform distribution in [0, stride_)
  // subtraction by one is over-low-safe, since stride is inside [1, 100]
  distribution_t::param_type param(0, stride_ - 1);
  dist_.param(param);
}

scan::scan_t sampler::operator()(const scan::scan_t &_in) noexcept {
  // check if sampling is required and if the input is valid
  if (stride_ == stride_min || stride_ >= _in.cols()) {
    return _in;
  }

  // get desired size and allocate memory
  const auto size = _in.cols() / stride_;
  scan::scan_t out(_in.rows(), size);

  // get a random start position in [0, stride_)
  auto ii = dist_(gen_);

  // populate the output
  for (size_t cc = 0; cc < size; ++cc, ii += stride_) {
    out.col(cc) = _in.col(ii);
  }
  return std::move(out);
}

constexpr size_t scan_matcher::max_iter;
constexpr size_t scan_matcher::min_iter;

scan_matcher::scan_matcher(ros::NodeHandle &_nh) :
        map_(_nh), is_converged_(_nh), sample_(_nh), knn_(nullptr) {
  ros::NodeHandle pnh(_nh, "icp");

  const auto raw = pnh.param("max_iter", static_cast<int>(max_iter));
  iter_ = cast_to_range(static_cast<size_t>(raw), min_iter, max_iter);
}

transform_t scan_matcher::operator()(const scan::scan_t &_scan) {
  transform_t out(transform_t::Identity());
  if (!map_.move_data(knn_) && !knn_) {
    I_SEE_PEE_WARN("no map data available");
    return out;
  }

  for (size_t ii = 0; ii < iter_; ++ii) {
    // get a random sample
    scan::scan_t random_scan = sample_(_scan);

    // update the data to the new position
    scan::scan_t data = out * random_scan;

    // get the problem instance
    matches m(data, *knn_);
    weight_t w = get_weights(m);

    // apply the icp algorithm
    const transform_t update = point_to_map(std::move(m), std::move(w));
    out = update * out;
    if (is_converged_(update)) {
      I_SEE_PEE_INFO("converged after " << ii);
      break;
    }
  }
  return out;
}

} // namespace icp

controller::controller(ros::NodeHandle &_nh) :
        icp_(_nh),
        scan_interface_(_nh, this),
        frame_handler_(_nh) {}

void controller::update(const scan::scan_t &_scan,
                        const stamp_t &_scan_stamp,
                        const std::string &_scan_frame) {
  // transform the data into global frame
  // todo add parameter not to run when there was no motion update
  const transform_t base_to_sensor = frame_handler_.get_base_to_sensor(_scan_frame);
  const transform_t map_to_base = frame_handler_.get_map_to_base();
  const transform_t map_to_sensor = map_to_base * base_to_sensor;

  scan::scan_t map_scan(map_to_sensor * _scan);

  // apply icp to get the correction
  const transform_t new_to_old = icp_(map_scan);

  // calculate the correction
  const transform_t new_to_base = new_to_old * map_to_base;
  const transform_t map_to_odom = frame_handler_.get_map_to_odom(new_to_base);

  // publish the transform
  frame_handler_.set_map_to_odom(map_to_odom, _scan_stamp);
}

} // namespace i_see_pee
