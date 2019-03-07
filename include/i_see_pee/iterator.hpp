#ifndef I_SEE_PEE_ITERATOR_HPP
#define I_SEE_PEE_ITERATOR_HPP

#include <Eigen/Dense>
#include <cmath>

namespace i_see_pee {

template<typename T>
struct centered_submap_iterator {

  using data_t = Eigen::Matrix<T, 2ul, 1ul>;
  using bound_t = Eigen::Matrix<size_t, 2ul, 1ul>;

  explicit centered_submap_iterator(const bound_t &_dim) noexcept :
      size_(_dim.cast<T>()), curr_(data_t::Zero()), prod_(size_.prod()) {
  }

  centered_submap_iterator &operator++() noexcept {
    ++curr_(1);
    if (curr_(1) >= size_(1)) {
      curr_(1) = 0;
      ++curr_(0);
    }
    return *this;
  }

  inline const data_t &operator*() const noexcept {
    return curr_;
  }

  inline const data_t *operator->() const noexcept {
    return &curr_;
  }

  inline bool past_end() const noexcept {
    return curr_(0) >= size_(0) || !prod_;
  }

private:
  data_t size_, curr_;
  size_t prod_;
};

template<typename T>
struct submap_iterator {

  using centered_submap_iterator_t = centered_submap_iterator<T>;
  using data_t = typename centered_submap_iterator_t::data_t;
  using bound_t = typename centered_submap_iterator_t::bound_t;

  submap_iterator() noexcept :
          iter_(bound_t::Zero()), begin_(data_t::Zero()), curr_(data_t::Zero()) {}

  submap_iterator(const data_t &_begin, const bound_t &_size) noexcept :
          iter_(_size), begin_(_begin), curr_(_begin) {}

  submap_iterator &operator++() noexcept {
    ++iter_;
    curr_ = begin_ + *iter_;
    return *this;
  }

  inline const data_t &operator*() const noexcept {
    return curr_;
  }

  inline const data_t *operator->() const noexcept {
    return &curr_;
  }

  inline bool past_end() const noexcept {
    return iter_.past_end();
  }

private:
  data_t begin_, curr_;
  centered_submap_iterator_t iter_;
};

template<typename T>
struct circle_iterator {

  using submap_iterator_t = submap_iterator<T>;
  using data_t = typename submap_iterator_t::data_t;
  using bound_t = typename submap_iterator_t::bound_t;

  circle_iterator(const data_t& _center, T _radius) noexcept :
          center_(_center), radius_(_radius) {
    // get the positive radius
    _radius = std::abs(_radius);

    // get the submap parameters with 1 pixel at the center
    const data_t rad(_radius, _radius);
    const data_t begin = _center - rad;
    const bound_t size = rad. template cast<size_t>() * 2 + bound_t::Ones();
    iter_ = submap_iterator_t(begin, size);

    // advance till valid position to allow instant access
    while(!valid(*iter_)){
      ++iter_;
    }
  }

  circle_iterator& operator++() noexcept {
    do{
      ++iter_;
    }
    while(!iter_.past_end() && !valid(*iter_));
    return *this;
  }

  inline const data_t &operator*() const noexcept {
    return *iter_;
  }

  inline const data_t *operator->() const noexcept {
    return iter_.operator->();
  }

  bool past_end() const noexcept {
    return iter_.past_end();
  }

private:

  bool valid(const data_t& _d) const noexcept {
    return (center_ - _d).norm() <= radius_;
  }

  const data_t center_;
  const T radius_;
  submap_iterator_t iter_;
};

} // namespace i_see_pee

#endif //I_SEE_PEE_ITERATOR_HPP
