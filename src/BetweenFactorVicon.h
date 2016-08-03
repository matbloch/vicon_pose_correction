#pragma once

#include <ostream>

#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class BetweenFactorVicon: public NoiseModelFactor2<VALUE, VALUE> {

    // Check that VALUE type is a testable Lie group
    BOOST_CONCEPT_ASSERT((IsTestable<VALUE>));
    BOOST_CONCEPT_ASSERT((IsLieGroup<VALUE>));

  public:

    typedef VALUE T;

  private:

    typedef BetweenFactorVicon<VALUE> This;
    typedef NoiseModelFactor2<VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */
    VALUE measured_tracker_; /** The tracker measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BetweenFactorVicon> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenFactorVicon() {}

    /** Constructor */
    BetweenFactorVicon(Key key1, Key key2, const VALUE& measured_vicon,
    	const VALUE& measured_tracker,
        const SharedNoiseModel& model
    ) :
      Base(model, key1, key2), measured_(measured_vicon), measured_tracker_(measured_tracker) {

    }

    virtual ~BetweenFactorVicon() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BetweenFactorVicon("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      traits<T>::Print(measured_tracker_, "  measured tracker: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol)
      	  	  && traits<T>::Equals(this->measured_, e->measured_, tol)
      	  	&& traits<T>::Equals(this->measured_tracker_, e->measured_tracker_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
  Vector evaluateError(const T& p1, const T& p2, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none) const {
	  /** p1: T_ARTKtoViconCam, p2: T_ARTKtoViconWorld */
      T T1 = traits<T>::Compose(p1, measured_.inverse(Eigen::Isometry), H1, boost::none);
      T T2 = traits<T>::Compose(measured_tracker_, T1, boost::none, boost::none);
      T hx = traits<T>::Compose(p2, T2, H2, boost::none);

      typename traits<T>::ChartJacobian::Fixed Hlocal;
      Vector rval = traits<T>::Local(measured_, hx, boost::none, (H1 || H2) ? &Hlocal : 0);
      if (H1) *H1 = Hlocal * (*H1);
      if (H2) *H2 = Hlocal * (*H2);

      return rval;
    }

    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** return the measured */
    const VALUE& measured_tracker() const {
      return measured_tracker_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 2;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(measured_tracker_);
    }
  }; // \class BetweenFactorVicon

  /// traits
  template<class VALUE>
  struct traits<BetweenFactorVicon<VALUE> > : public Testable<BetweenFactorVicon<VALUE> > {};

} /// namespace gtsam
