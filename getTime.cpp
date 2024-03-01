namespace GETTIME{

  std::vector<int> func(int a, float b) {
        printf("func:%f", b);
        return {};
    }

    //全局函数的输出可以用void代替F自动推断返回类型；成员函数必须加Ｆ
    template<typename F, typename... Args>
    class getFuncTime_g {
    public:
        using funcPtr = std::function<F(Args... args)>;//全局非成员函数指针用法

        static void getTime(funcPtr func, Args... args) {
            auto start = clock();
            func(args...);
            std::cout << "time use: " << (clock() - start) << std::endl;
        }
    };

    std::vector<int> A::func(int a, float b) {
        printf("A::func:%f", b);
        return {};
    }

    template<typename F, typename... Args>
    class getFuncTime_m {
    public:
        template<typename C>
        //成员函数指针A::test1需要通过this绑定传入
        static void getTime(C *obj, F(C::*func)(Args...), Args... args) {
            auto start = clock();
            (obj->*func)(args...);
            std::cout << "time use: " << (clock() - start) << std::endl;
        }

    };


    void A::test() {
        int val{5};
        float b{4.7};
        getFuncTime_m<std::vector<int>, int, float>::getTime(this, &A::func, val, b);
        getFuncTime_g<std::vector<int>, int, float>::getTime(GETTIME::func, val, b);
    }
}
