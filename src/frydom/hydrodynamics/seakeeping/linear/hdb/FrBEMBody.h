// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRBEMBODY_H
#define FRYDOM_FRBEMBODY_H


#include "frydom/core/math/FrVector.h"

#include "MathUtils/LookupTable2D.h"
#include "MathUtils/Interp1d.h"
#include "MathUtils/Matrix.h"


namespace frydom {

  // Forward declaration
  class FrHydroDB;

  /**
   * \class FrBEMMode
   * \brief
   */
  class FrBEMMode {

   public:
    enum TYPE {
      LINEAR,
      ANGULAR
    };

   private:
    TYPE m_type;
    Direction m_direction;
    Position m_position;
    bool m_active = true;

   public:
    FrBEMMode() = default;

    void SetTypeLINEAR() { m_type = LINEAR; }

    void SetTypeANGULAR() { m_type = ANGULAR; }

    TYPE GetType() const { return m_type; }

    void SetDirection(Direction &direction) { m_direction = direction; }

    Direction GetDirection() const { return m_direction; }

    void SetPointPosition(Position &position) { m_position = position; }

    Position GetPointPosition() const { return m_position; }

    // TODO : flag pour le couplage FrBody/FrBEMBody (a voir si conservation necessaire)
    void Activate() { m_active = true; }

    void Deactivate() { m_active = false; }

    bool IsActive() const { return m_active; }

  };

  typedef FrBEMMode FrBEMForceMode;
  typedef FrBEMMode FrBEMMotionMode;


  class FrBEMDOFMask {

   private:
    unsigned int m_nbDOF;                   ///< Number of degree of freedom of the body
    std::vector<int> m_listDOF;             ///< List of degree of freedom of the body
    mathutils::Vector6d<bool> m_mask;       ///< Mask applied on the degree of freedom
    mathutils::MatrixMN<double> m_matrix;   ///<

   public:
    /// Default constructor of the DOF Mask
    FrBEMDOFMask() {}

    /// Define the DOF mask from vector
    /// \param mask DOF Mask vector
    void SetMask(mathutils::Vector6d<int> mask);

    /// Return the DOF Mask
    /// \return DOF Mask
    mathutils::Vector6d<bool> GetMask() const;

    /// Return the mask for a specific degree of freedom
    /// \param imode Degree of freedom
    /// \return True if the mask is active, False otherwise
    bool GetMask(unsigned int imode) const { return m_mask[imode]; }

    ///
    /// \return
    mathutils::MatrixMN<double> GetMatrix() const;

    /// Return the number of activated DOF in the mask
    /// \return Number of activated DOF
    unsigned int GetNbMode() const { return m_nbDOF; }

    /// Return the list of activated DOF
    /// \return List of activated DOF
    std::vector<int> GetListDOF() const { return m_listDOF; }
  };


  struct FrWaveDriftPolarData {
    std::vector<double> m_angles;       ///< Wave directions from the head
    std::vector<double> m_freqs;        ///< Wave frequency discretization for the wave drift coefficient
    std::vector<double> m_data;         ///< Wave drift coefficients value
    std::unique_ptr<mathutils::LookupTable2d<double>> m_table;  ///< 2D interpolation table

    /// Default constructor
    FrWaveDriftPolarData();

    /// Define the list of wave direction for the polar coefficient definition
    /// \param angles List of the direction angles
    void SetAngles(const std::vector<double> &angles);

    /// Define the list of the wave frequencies fro the polar coefficient definition
    /// \param freqs List of the wave frequencies
    void SetFrequencies(const std::vector<double> &freqs);

    /// Adding new data with wave drift coefficients
    /// \param name Name of the data
    /// \param coeffs >Wave drift coefficient values
    void AddData(std::string &name, std::vector<double> coeffs);

    /// Evaluate the value of the wave drift coefficient
    /// \param name Name of the data
    /// \param x Wave directions with respect to the heading angle
    /// \param y Wave frequency
    /// \return Interpolated value of the wave drift coefficient
    double Eval(const std::string &name, double x, double y) const;

    /// Return true if wave drift coefficient in "surge" is present
    bool HasSurge() const;

    /// Return true if wave drift coefficient in "sway" is present
    bool HasSway() const;

    /// Return true if wave drift coefficient in "heave" is present
    bool HasHeave() const;

    /// Return true if wave drift coefficient in "pitch" is present
    bool HasPitch() const;

    /// Return true if wave drift coefficient in "roll" is present
    bool HasRoll() const;

    /// Return true if wave drift coefficient in "yaw" is present
    bool HasYaw() const;
  };

  /**
   * \class FrBEMBody
   * \brief Class for defining a body subject to hydrodynamic loads using a BEM solver.
   */
  class FrBEMBody {

   private:

    FrHydroDB *m_HDB;       ///< HDB from which BEM data are extracted
    unsigned int m_id;      ///< ID of the BEM Body
    std::string m_name;     ///< Name of the body
    Position m_position;    ///< Position of the body from HDB

    FrBEMDOFMask m_forceMask;  ///< Mask applied on the force
    FrBEMDOFMask m_motionMask; ///< Mask applied on the DOF

    std::vector<FrBEMForceMode> m_forceModes;   ///< List of activated force modes
    std::vector<FrBEMMotionMode> m_motionModes; ///< List of activated motion modes

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_excitationMask;   ///<
    std::vector<Eigen::MatrixXcd> m_excitation;         ///< Complex coefficient of the excitation force
    std::vector<Eigen::MatrixXcd> m_froudeKrylov;       ///< Complex coefficient of the froude-krylov force
    std::vector<Eigen::MatrixXcd> m_diffraction;        ///< Complex coefficient of the diffraction force

    std::vector<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> m_radiationMask;   ///<
    std::unordered_map<FrBEMBody *, mathutils::Matrix66<double>> m_infiniteAddedMass;    ///< Infinite added mass for each body
    std::unordered_map<FrBEMBody *, std::vector<std::shared_ptr<mathutils::Interp1d<double, mathutils::Vector6d<double>>> >> m_interpK; ///< Impulse response function interpolator
    std::unordered_map<FrBEMBody *, std::vector<std::shared_ptr<mathutils::Interp1d<double, mathutils::Vector6d<double>>> >> m_interpKu; ///< Impulse response function speed dependent interpolator

    std::vector<std::vector<mathutils::Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;   ///<

    std::shared_ptr<FrWaveDriftPolarData> m_waveDrift;  ///< List of wave drift coefficients

    mathutils::Matrix33<double> m_hydrostaticStiffnessMatrix;   ///< Hydrostatic matrix

   public:
    /// Constructor the BEM body database
    /// \param id Unique integer identifier of the database
    /// \param name Name of the BEM body
    /// \param HDB Hydrodynamic database
    FrBEMBody(unsigned int id, std::string name, FrHydroDB *HDB)
        : m_id(id), m_name(name), m_HDB(HDB) {}

    /// Return the name of the BEM body database
    /// \return Name of the BEM body
    std::string GetName() const { return m_name; }

    /// Return the identifier of the BEM Body
    /// \return Identifier
    unsigned int GetID() const { return m_id; }

    /// Return the number of discrete frequencies in the database
    /// \return Number of discret frequencies
    unsigned int GetNbFrequencies() const;

    /// Return the list of discrete frequencies
    /// \return List of the discrete frequencies
    std::vector<double> GetFrequencies() const;

    /// Return the number of discrete wave direction in the database
    /// \return Number of discrete direction
    unsigned int GetNbWaveDirections() const;

    /// Return the discrete wave direction angles
    /// \param angleUnit Unit of the wave direction
    /// \param fc Frame convention
    /// \return Discrete wave direction
    std::vector<double> GetWaveDirections(mathutils::ANGLE_UNIT angleUnit, FRAME_CONVENTION fc) const;

    /// Return the number of bodies in interaction is the current BEM body
    /// \return Number of bodies in interaction
    unsigned int GetNbBodies() const;

    /// Return the number time sample in the database
    /// \return Number of time sample in the database
    unsigned int GetNbTimeSamples() const;

    /// Method to initialize the database
    void Initialize();

    //
    // Mask
    //

    /// Define the mask on the force components
    /// \param mask Mask on the force components
    void SetForceMask(mathutils::Vector6d<int> mask);

    /// Define the mask on the DOF of the body
    /// \param mask Mask on the DOF of the body
    void SetMotionMask(mathutils::Vector6d<int> mask);

    /// Return the mask matrix applied on force components
    /// \return Mask matrix
    mathutils::MatrixMN<double> GetForceMaskMatrix() const { return m_forceMask.GetMatrix(); }

    /// Return the mask matrix applied on motion DOF
    /// \return Mask matrix
    mathutils::MatrixMN<double> GetMotionMaskMatrix() const { return m_motionMask.GetMatrix(); }

    /// Return the number of force mode
    /// \return Number of force mode
    unsigned int GetNbForceMode() const;

    /// Return the number of motion mode
    /// \return Number of motion mode
    unsigned int GetNbMotionMode() const;

    /// Return the mask value applied on a specific motion mode
    /// \param imotion Index of motion
    /// \return Mask on the motion mode
    bool GetMotionMask(unsigned int imotion) const { return m_motionMask.GetMask(imotion); }

    /// Return the mask value applied on a specific motion mode
    /// \param iforce Index of force
    /// \return Mask on the force mode
    bool GetForceMask(unsigned int iforce) const { return m_forceMask.GetMask(iforce); }

    /// Return the list of DOF of the BEM Body
    /// \return List of DOF
    std::vector<int> GetListDOF() const { return m_motionMask.GetListDOF(); }

    //
    // Generalized modes
    //

    //unsigned int GetNbForceMode() const { return (uint)m_forceModes.size(); }

    //unsigned int GetNbMotionMode() const { return (uint)m_motionModes.size(); }

    /// Return the force mode definition
    /// \param imode Mode index
    /// \return Mode definition
    FrBEMForceMode *GetForceMode(unsigned int imode);

    /// Return the motion mode definition
    /// \param imode Mode index
    /// \return Mode definition
    FrBEMMotionMode *GetMotionMode(unsigned int imode);

    /// Adding a new force mode to the BEM body database
    /// \param mode Force mode
    void AddForceMode(FrBEMForceMode &mode);

    /// Adding a new motion mode to the BEM body database
    /// \param mode Motion mode
    void AddMotionMode(FrBEMMotionMode &mode);

    //
    // Setters
    //

    /// Define the position of body stored in BEM body database
    /// \param position Position of the body
    void SetPosition(Position position) { m_position = position; }

    /// Set the complex matrix of the diffraction coefficient
    /// \param iangle Corresponding wave direction
    /// \param diffractionMatrix Complex matrix of the diffraction coefficient
    void SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd &diffractionMatrix);

    /// Set the complex matrix of the froude-krylov coefficient
    /// \param iangle Corresponding wave direction
    /// \param froudeKrylovMatrix Complex matrix of the diffraction coefficient
    void SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd &froudeKrylovMatrix);

    /// Set the complex matrix of the wave excitation coefficient
    /// \param iangle Corresponding wave direction
    /// \param excitationMatrix Complex matrix of the wave excitation coefficients
    void SetExcitation(unsigned int iangle, const Eigen::MatrixXcd &excitationMatrix);

    /// Compute the excitation loads from the diffraction loads and the Froude-Krylov loads.
    void ComputeExcitation();

    /// Set the infinite added mass of the BEM body with respect to the motion of another BEM body
    /// \param BEMBodyMotion BEM body to which the motion is considered
    /// \param CMInf Infinite added mass matrix
    void SetInfiniteAddedMass(FrBEMBody *BEMBodyMotion, const Eigen::MatrixXd &CMInf);

    /// Set the impulse response function of the BEM body with respect to the motion of another BEM body
    /// \param BEMBodyMotionBEM body to which the motion is considered
    /// \param listIRF List of impulse response function (size nforce x ntime) for each DOF
    void SetImpulseResponseFunctionK(FrBEMBody *BEMBodyMotion, const std::vector<Eigen::MatrixXd> &listIRF);

    /// Set the impulse response function (steady-speed dependent) of the BEM body with respect to the motion of another BEM body
    /// \param BEMBodyMotion BEM body to which the motion is considered
    /// \param listIRF List of impulse response function (size nforce x ntime) for each DOF
    void SetImpulseResponseFunctionKu(FrBEMBody *BEMBodyMotion, const std::vector<Eigen::MatrixXd> &listIRF);

    /// Set the hydrostatic stiffness Matrix
    /// \param hydrostaticStiffnessMatrix Hydrostatic stiffness matrix
    void SetStiffnessMatrix(const mathutils::Matrix33<double> &hydrostaticStiffnessMatrix);

    /// Set the hydrostatic stiffness Matrix
    /// \param hydrostaticStiffnessMatrix Hydrostatic stiffness matrix
    void SetStiffnessMatrix(const mathutils::Matrix66<double> &hydrostaticStiffnessMatrix);

    /// Set the wave drift coefficient database
    void SetWaveDrift();
    //
    // Getters
    //

    Eigen::MatrixXcd GetDiffraction(unsigned int iangle) const;

    Eigen::VectorXcd GetDiffraction(unsigned int iangle, unsigned int iforce) const;

    Eigen::MatrixXcd GetFroudeKrylov(unsigned int iangle) const;

    Eigen::VectorXcd GetFroudeKrylov(unsigned int iangle, unsigned int iforce) const;

    Eigen::MatrixXcd GetExcitation(unsigned int iangle) const;

    Eigen::VectorXcd GetExcitation(unsigned int iangle, unsigned int iforce) const;

    mathutils::Matrix66<double> GetInfiniteAddedMass(FrBEMBody *BEMBodyMotion) const;

    mathutils::Matrix66<double> GetSelfInfiniteAddedMass();

    mathutils::Interp1d<double, mathutils::Vector6d<double>> *
    GetIRFInterpolatorK(FrBEMBody *BEMBodyMotion, unsigned int idof);

    mathutils::Interp1d<double, mathutils::Vector6d<double>> *
    GetIRFInterpolatorKu(FrBEMBody *BEMBodyMotion, unsigned int idof);

    mathutils::Matrix33<double> GetHydrostaticStiffnessMatrix() const { return m_hydrostaticStiffnessMatrix; }

    std::shared_ptr<FrWaveDriftPolarData> GetWaveDrift() const;

    void BuildDiffractionInterpolators();


  };

}  // end namespace frydom


#endif //FRYDOM_FRBEMBODY_H
