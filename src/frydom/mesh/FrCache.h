//
// Created by frongere on 24/05/18.
//

#ifndef FRYDOM_DICE_DCACHE_H
#define FRYDOM_DICE_DCACHE_H

namespace frydom {

    template <typename T>
    class FrCache {

    private:
        bool m_valid;
        T m_data;

    public:
        FrCache() : m_valid(false), m_data() {}

        FrCache(const T& data) : m_valid(true), m_data(data) {}

        T& Get() {
            assert(IsValid());
            return m_data;
        }

        const T& Get() const {
            assert(IsValid());
            return m_data;
        }

        bool IsValid() const { return m_valid; }

        void Invalidate() { m_valid = false; }

        void Set(const T& data) {
            m_data = data;
            m_valid = true;
        }

        FrCache<T>& operator =(const T& data) {
            Set(data);
        }

        operator T&() { return m_data; }
        operator const T&() const { return m_data; }

        operator bool() const { return m_valid; }

    };

}  // end namespace frydom

#endif //FRYDOM_DICE_DCACHE_H
