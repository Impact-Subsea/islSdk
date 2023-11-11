#ifndef SIGSLOT_H_
#define SIGSLOT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <functional>
#include <vector>

//--------------------------------------- Class Definition -----------------------------------------
namespace IslSdk
{
    /**
    * @brief This class is used to create a callback.
    * This is a lightweight class that is used to create a callback. The callback can be a member function or a static function.
    * It only stores one function pointer.
    */
    template<typename... Args>
    class Callback
    {
    private:
        std::function<void(Args...)> callback;
    public:
        Callback() : callback(nullptr) {}
        template<typename T>  Callback(T* inst, void (T::* func)(Args...)) : callback([=](Args... args) {(inst->*func)(args...); }) {}

        template<typename T> void connect(T* inst, void (T::* func)(Args...))
        {
            callback = ([=](Args... args) {(inst->*func)(args...); });
        }

        void connect(void (*func)(Args...))
        {
            callback = func;
        }

        void disconnect()
        {
            callback = nullptr;
        }

        void operator()(Args... p)
        {
            if (callback)
            {
                callback(p...);
            }
        }
    };

    template<typename... Args>
    class Signal;                   // Forward declaration for slot

    /**
    * @brief This class is used to connect a function to a signal.
    * It is used to connect a function to a signal. The function can be a member function or a static function.
    * The function can be connected to multiple signals of the same type.
    * When the slot is destroyed, it automatically disconnects from all signals.
    */
    template<typename... Args>
    class Slot
    {
    private:
        std::function<void(Args...)> callback;
        std::vector<Signal<Args...>*> sigArray;

        void remove(const Signal<Args...>* sig)
        {
            for (size_t i = 0; i < sigArray.size(); i++)
            {
                if (sigArray[i] == sig)
                {
                    sigArray.erase(sigArray.begin() + i);
                    break;
                }
            }
        }

    public:
        template<typename T>

        /**
        * @brief Constructor.
        * @param inst The instance of the class.
        * @param func The member callback function.
        */
        Slot(T* inst, void (T::* func)(Args...)) : callback([=](Args... args) {(inst->*func)(args...); }) {}

        /**
        * @brief Constructor.
        * @param func The static callback function.
        */
        Slot(std::function<void(Args...)> func) : callback(func) {}

        /**
        * @brief Destructor.
        * Automatically disconnects from all signals.
        */
        ~Slot()
        {
            while (sigArray.size())
            {
                sigArray[0]->disconnect(*this);
            }
        }

        void operator()(Args... p)
        {
            callback(p...);
        }
        friend class Signal<Args...>;
    };

#ifndef PYTHON_WRAPPER

    /**
    * @brief This class is used to create a signal.
    * The signal connects to slots, which contain the function pointer, and calls them when the signal is called.
    */
    template<typename... Args>
    class Signal
    {
    private:
        std::vector<Slot<Args...>*> slotArray;
        std::function<void(uint_t)> callback;
        int_t iter;

        int_t find(const Slot<Args...>& slot)
        {
            for (uint_t i = 0; i < slotArray.size(); i++)
            {
                if (slotArray[i] == &slot)
                {
                    return i;
                }
            }
            return -1;
        }

        bool_t addSlot(Slot<Args...>& slot)
        {
            if (find(slot) < 0)
            {
                slot.sigArray.push_back(this);
                slotArray.push_back(&slot);

                if (callback != nullptr)
                {
                    callback(slotArray.size());
                }
            }
            return false;
        }

    public:

        /**
        *
        * @brief Default constructor.
        */
        constexpr Signal() : callback(nullptr), iter(-1){}

        /**
        * @brief Constructor which take a callback function for subscriber count changes.
        * @param inst The instance of the class.
        * @param func The member callback function.
        */
        template<typename T>  Signal(T* inst, void (T::* func)(uint_t)) : callback([=](uint_t count) {(inst->*func)(count); }), iter(-1) {}
        Signal(std::function<void(uint_t)> func) : callback(func), iter(-1) {}

        // copy/move constructor
        Signal(const Signal&) = delete;
        Signal(Signal&&) noexcept = delete;

        // copy/move assignment
        Signal& operator=(const Signal&) = delete;
        Signal& operator=(Signal&&) noexcept = delete;

        /**
        * @brief Destructor.
        * Automatically disconnects from all slots.
        */
        ~Signal()
        {
            for (size_t i = 0; i < slotArray.size(); i++)
            {
                slotArray[i]->remove(this);
            }
        }

        /**
        * @brief Sets a member callback function that is called when the number of subscribers changes.
        * @param inst The instance of the class.
        * @param func The member callback function.
        */
        template<typename T> void setSubscribersChangedCallback(T* inst, void (T::* func)(uint_t))
        {
            callback = ([=](uint_t count) {(inst->*func)(count); });
        }

        /**
        * @brief Sets a static callback function that is called when the number of subscribers changes.
        * @param func The callback function.
        */
        void setSubscribersChangedCallback(void (*func)(uint_t))
        {
            callback = func;
        }

        /**
        * @brief Connects a slot to a signal.
        * @param slot The slot to connect.
        */
        void connect(Slot<Args...>& slot)
        {
            addSlot(slot);
        }

        /**
        * @brief Disconnects a slot from a signal.
        * @param slot The slot to disconnect.
        */
        void disconnect(Slot<Args...>& slot)
        {
            int_t i = find(slot);

            if (i >= 0)
            {
                if (iter >= i)
                {
                    iter--;
                }
                slotArray[i]->remove(this);
                slotArray.erase(slotArray.begin() + i);

                if (callback != nullptr)
                {
                    callback(slotArray.size());
                }
            }
        }

        /**
        * @brief Checks if the signal has subscribers.
        * @return True if the signal has subscribers.
        */
        bool_t hasSubscribers(void)
        {
            return slotArray.size() != 0;
        }

        /**
        * @brief Triggers the calling of the connected slots.
        * @param p The parameters to pass to the slots.
        */
        void operator()(Args... p)
        {
            for (iter = 0; iter < static_cast<int_t>(slotArray.size()); iter++)
            {
                (*slotArray[iter])(p...);
            }
            iter = -1;
        }
    };

#else

    class BaseSignal
    {
    public:
        BaseSignal(std::function<void(uint_t)> func) : callback(func), callbackEnable(false) {}
        ~BaseSignal() {}
        std::function<void(uint_t)> callback;
        bool_t callbackEnable;

        void pyEnableCallback(bool_t enable)
        {
            callbackEnable = enable;

            if (callback != nullptr)
            {
                callback(callbackEnable);
            }
        }
    };

    template<typename... Args>
    class Signal : public BaseSignal
    {
    private:
        std::vector<Slot<Args...>*> slotArray;
        void* pyObj;
        void(*pyCallback)(void* pyObj, Args...);
        int_t iter;

        int_t find(Slot<Args...>& slot)
        {
            for (size_t i = 0; i < slotArray.size(); i++)
            {
                if (slotArray[i] == &slot)
                {
                    return i;
                }
            }
            return -1;
        }

        bool_t addSlot(Slot<Args...>& slot)
        {
            if (find(slot) < 0)
            {
                slot.sigArray.push_back(this);
                slotArray.push_back(&slot);

                if (callback != nullptr)
                {
                    callback(slotArray.size() || callbackEnable);
                }
            }
            return false;
        }

    public:

        // default constructor
        constexpr Signal() : pyObj(nullptr), pyCallback(nullptr), BaseSignal(nullptr), iter(-1) {}
        template<typename T>  Signal(T* inst, void (T::* func)(uint_t)) : pyObj(nullptr), pyCallback(nullptr), BaseSignal([=](uint_t count) {(inst->*func)(count); }), iter(-1) {}
        Signal(std::function<void(uint_t)> func) : pyObj(nullptr), pyCallback(nullptr), BaseSignal(func), iter(-1) {}

        // copy/move constructor
        Signal(const Signal&) = delete;
        Signal(Signal&&) noexcept = delete;

        // copy/move assignment
        Signal& operator=(const Signal&) = delete;
        Signal& operator=(Signal&&) noexcept = delete;

        ~Signal()
        {
            for (size_t i = 0; i < slotArray.size(); i++)
            {
                slotArray[i]->remove(this);
            }
        }

        template<typename T> void setSubscribersChangedCallback(T* inst, void (T::* func)(uint_t))
        {
            callback = ([=](uint_t count) {(inst->*func)(count); });
        }

        void setSubscribersChangedCallback(void (*func)(uint_t))
        {
            callback = func;
        }

        void connect(Slot<Args...>& slot)
        {
            addSlot(slot);
        }

        void disconnect(Slot<Args...>& slot)
        {
            int_t i = find(slot);

            if (i >= 0)
            {
                if (iter >= i)
                {
                    iter--;
                }
                slotArray[i]->remove(this);
                slotArray.erase(slotArray.begin() + i);

                if (callback != nullptr)
                {
                    callback(slotArray.size() || callbackEnable);
                }
            }
        }

        bool_t hasSubscribers(void)
        {
            return slotArray.size() != 0 || callbackEnable;
        }

        void pySetCallback(void* obj, void(*func)(void* pyObj, Args...))
        {
            pyObj = obj;
            pyCallback = func;
        }

        void operator()(Args... p)
        {
            for (iter = 0; iter < slotArray.size(); iter++)
            {
                (*slotArray[iter])(p...);
            }
            iter = -1;

            if (callbackEnable && pyCallback)
            {
                pyCallback(pyObj, p...);
            }
        }
    };
#endif
}
//--------------------------------------------------------------------------------------------------
#endif
