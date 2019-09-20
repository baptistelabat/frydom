//
// Created by frongere on 12/10/18.
//

#ifndef FRYDOM_FRFUNCTION_H
#define FRYDOM_FRFUNCTION_H


#include "chrono/motion_functions/ChFunction.h"

#include "frydom/core/common/FrObject.h"


namespace frydom {


    // Forward declaration
    class FrFunctionBase;

    namespace internal {

        struct FrChronoFunctionWrapper : public chrono::ChFunction {

            FrFunctionBase* m_frydomFunction;

            explicit FrChronoFunctionWrapper(FrFunctionBase* frydomFunction);

            FrChronoFunctionWrapper* Clone() const override;

            double Get_y(double x) const override;

            double Get_y_dx(double x) const override;

            double Get_y_dxdx(double x) const override;

        };

    }  // end namespace frydom::internal

    // Forward declaration
    class FrUnaryOpFunction;
    class FrAddFunction;
    class FrSubFunction;
    class FrMulFunction;
    class FrDivFunction;
    class FrCompFunction;


    /*
     * FrFunctionBase
     */

    class FrFunctionBase {

    private:

//        std::shared_ptr<internal::FrChronoFunctionWrapper> m_chronoFunction;  // TODO : deleguer cet attribut a du chrono interface qu'on placera en interne ?

        bool m_isActive = true; // TODO : retirer ?

    protected:

        FrFunctionBase* m_function; /// The function on which we apply the current function

        double m_xOffset = 0.;  // TODO : supprimer et faire plutot une fonciton retard qui agit sur x...

        // Cache
        mutable double c_x = INFINITY;
        mutable double c_y;
        mutable double c_y_dx;
        mutable double c_y_dxdx;

    public:

        FrFunctionBase();

        ~FrFunctionBase();


        /// Constructor specifying the function on which we apply the current function
        FrFunctionBase(const FrFunctionBase& other);

        /// Virtual copy constructor
        virtual FrFunctionBase* Clone() const = 0;

        bool IsActive() const;  // TODO : supprimer cette fonctionnalite, repercuter dans le reste du code...
        void SetActive(bool active);

        void SetXOffset(double xOffset);
        double GetXOffset() const;

        double Get_y(double x) const;
        double Get_y_dx(double x) const;
        double Get_y_dxdx(double x) const;

//        void Initialize() override;

        double operator()(double x) const;

        // TODO : ajouter operateurs pour ajout de scalaire...

        virtual std::string GetRepr() const = 0;

        void WriteToGnuPlotFile(double xmin, double xmax, double dx, std::string filename = "functionOutput") const;

        /*
         * Operators
         */

        /*
         * Unary operations on a function
         */

        /// Negate a function
        FrUnaryOpFunction operator-();
        FrUnaryOpFunction operator+();

        /*
         * Binary operations with another function
         */

        /// Add two functions
        FrAddFunction operator+(const FrFunctionBase& other);

        /// Substract two functions
        FrSubFunction operator-(const FrFunctionBase& other);

        /// Multiplpy two functions
        FrMulFunction operator*(const FrFunctionBase& other);

        /// Divide two functions
        FrDivFunction operator/(const FrFunctionBase& other);

        /// Compose two functions -> this(other(x))
        FrCompFunction operator<<(const FrFunctionBase& other);

        /*
         * Binary operations with a scalar on the right
         */

        /// Right multiply a function by a scalar
        FrMulFunction operator*(double alpha);

        /// Right divide a function by a scalar
        FrDivFunction operator/(double alpha);

        /// Add a scalar to the function to the right
        FrAddFunction operator+(double alpha);

        /// Substract a scalar to the function to the right
        FrSubFunction operator-(double alpha);

        /*
         * Inner operations with another function
         */

        /// Add an other function to this function
        void operator+=(const FrFunctionBase& other);

        /// Substract an other function to this function
        void operator-=(const FrFunctionBase& other);

        /// Multiply this function by another function
        void operator*=(const FrFunctionBase& other);

        /// Divide this function by another function
        void operator/=(const FrFunctionBase& other);

        /*
         * Inner operations with a scalar
         */

        /// Add an other function to this function
        void operator+=(double alpha);

        /// Substract an other function to this function
        void operator-=(double alpha);

        /// Multiply this function by another function
        void operator*=(double alpha);

        /// Divide this function by another function
        void operator/=(double alpha);


    protected:

        virtual void Eval(double x) const = 0;

        double Estimate_y_dx(double x) const;

        double Estimate_y_dxdx(double x) const;

        inline bool IsEval(double x) const {
            return c_x == x;
        }

    };

    /*
     * Binary operations of functions with a scalar on the left
     */

    /// Add a scalar to the function to the left
    FrAddFunction operator+(double alpha, const FrFunctionBase& function);

    /// Add a scalar to the function to the left
    FrSubFunction operator-(double alpha, const FrFunctionBase& function);

    /// Left multiply a function by a scalar
    FrMulFunction operator*(double alpha, const FrFunctionBase& function);

    /// Inverse a function and multiply by a scalar
    FrDivFunction operator/(double alpha, const FrFunctionBase& function);


    namespace internal {

        /// This class is used internally to add a chrono function object to be added into chrono objects that accept
        /// a ChFunction
        class FrFunctionChronoInterface { // : public FrFunctionBase { // TODOD : doit-on heriter de FrFunctionBase ? -> oui !

        private:
            std::shared_ptr<internal::FrChronoFunctionWrapper> m_chronoFunction;
            FrFunctionBase* m_function; /// The function on which we apply the current function

        public:
            explicit FrFunctionChronoInterface(const FrFunctionBase& frydomFunction);


//        protected:
            std::shared_ptr<internal::FrChronoFunctionWrapper> GetChronoFunction();

        };

    }


    class FrVarXFunction : public FrFunctionBase {

    private:
        std::string m_varname = "x";

    public:
        FrVarXFunction();
        FrVarXFunction(std::string varname);
        FrVarXFunction(const FrVarXFunction& other);
        FrVarXFunction* Clone() const override;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "VarXFunction"; }

    protected:
        void Eval(double x) const;

    };

    /// Create a symbolic variable (default variable name is 'x')
    FrVarXFunction new_var();

    /// Create a symbolic variables named "varname"
    FrVarXFunction new_var(std::string varname);


    /// The constant function
    class FrConstantFunction : public FrFunctionBase {

    public:
        explicit FrConstantFunction(double scalar);
        FrConstantFunction(const FrConstantFunction& other);
        FrConstantFunction* Clone() const;

        void Set(double scalar);
        double Get() const;
        double operator()() const;
        double& operator()();
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "ConstantFunction"; }

    protected:
        void Eval(double x) const override;

    };



    /*
     * Results of unary operators
     */
    class FrUnaryOpFunction : public FrFunctionBase {

    private:
        bool m_negate = false;

    public:
        FrUnaryOpFunction(const FrFunctionBase& function, bool negate);
        FrUnaryOpFunction(const FrUnaryOpFunction& other);
        FrUnaryOpFunction* Clone() const;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "UnaryOpFunction"; }

    protected:
        void Eval(double x) const override;

    };


    /*
     * Results of binary operators
     */


    /// Base class for functions that result from a binary operation (abstract)
    class FrBinaryOpFunction : public FrFunctionBase {

    protected:
        FrFunctionBase* m_rightFunction;

    public:
        FrBinaryOpFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrBinaryOpFunction(const FrBinaryOpFunction& other);

    protected:

        inline void EvalFunctions(double& u, double& u_dx, double& u_dxdx, double& v, double& v_dx, double& v_dxdx) const {
            u = m_function->Get_y(c_x);
            v = m_rightFunction->Get_y(c_x);

            u_dx = m_function->Get_y_dx(c_x);
            v_dx = m_rightFunction->Get_y_dx(c_x);

            u_dxdx = m_function->Get_y_dxdx(c_x);
            v_dxdx = m_rightFunction->Get_y_dxdx(c_x);
        }

    };



    /// Class that result from adding two functions
    class FrAddFunction : public FrBinaryOpFunction {

    public:
        FrAddFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrAddFunction(const FrAddFunction& other);
        FrAddFunction* Clone() const;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "AddFunction"; }

    protected:
        void Eval(double x) const override;

    };
    /// Class that result from subtracting two functions
    class FrSubFunction : public FrBinaryOpFunction {

    public:
        FrSubFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrSubFunction(const FrSubFunction& other);
        FrSubFunction* Clone() const;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "SubFunction"; }

    protected:
        void Eval(double x) const override;

    };

    /// Class that result from multiplying two functions
    class FrMulFunction : public FrBinaryOpFunction {

    public:
        FrMulFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrMulFunction(const FrMulFunction& other);
        FrMulFunction* Clone() const;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "MulFunction"; }

    protected:
        void Eval(double x) const override;

    };

    /// Class that result from dividing two functions
    class FrDivFunction : public FrBinaryOpFunction {

    public:
        FrDivFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrDivFunction(const FrDivFunction& other);
        FrDivFunction* Clone() const;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "divFunction"; }

    protected:
        void Eval(double x) const override;

    };

    /// Class that result from composing two functions
    class FrCompFunction : public FrBinaryOpFunction {

    public:
        FrCompFunction(const FrFunctionBase& f1, const FrFunctionBase& f2);
        FrCompFunction(const FrCompFunction& other);
        FrCompFunction* Clone() const;
        std::string GetRepr() const override;

//        std::string GetTypeName() const override { return "CompFunction"; }

    protected:
        void Eval(double x) const override;

    };


}  // end namespace frydom

#endif //FRYDOM_FRFUNCTION_H
