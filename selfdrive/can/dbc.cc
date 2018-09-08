#include <string>
#include <vector>

#include "common.h"

namespace {

std::vector<const DBC*>& get_dbcs() {
  static std::vector<const DBC*> vec;
  return vec;
}

}

const DBC* dbc_lookup(const std::string& dbc_name) {
  // Print in dbc.cc string
  std::cout << "print from dbc.cc get_dbcs()" << std::endl;
  //cout<<"print from dbc.cc get_dbcs()";
  std::cout <<get_dbcs()<< std::endl;
  //cout<<get_dbcs();
  for (const auto& dbci : get_dbcs()) {
    if (dbc_name == dbci->name) {
      return dbci;
    }
  }
  return NULL;
}

void dbc_register(const DBC* dbc) {
  get_dbcs().push_back(dbc);
}

extern "C" {
  const DBC* dbc_lookup(const char* dbc_name) {
    return dbc_lookup(std::string(dbc_name));
  }
}
