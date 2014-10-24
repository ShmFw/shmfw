#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
using namespace std;

class Test {
    int a, b;
public:
    Test(): a(0), b(0) { }
    Test(const Test &test): a(test.a), b(test.b) { }
    ~Test() { }

    void print() {
        cout << "A = " << a << endl;
        cout << "B = " << b << endl;
    }

    friend ostream& operator<< (ostream &output, const Test &test) {
        output << test.a << test.b;
        return output;
    }

    bool fromString(const string &str){
      int start = str.find("[");
      int end = str.find_last_of("]");
      string data = str.substr(start+1, end-1);
      boost::erase_all(data, " ");
      sscanf(data.c_str(), "%d,%d", &a, &b);
      return true;
    }
    
    friend std::istream& operator>>(std::istream &input, Test &test)
    {
        std::string str;
        getline (input, str);
        test.fromString(str);
        return input;
    }
};

int main(int argc, char* argv[] ) {
  Test test;
  test.print();
    try {
        test = boost::lexical_cast<Test>("[10, 23]");
    } catch(std::exception &e) {
        cout << e.what() << endl;
    }
  test.print();
    return 0;
}
