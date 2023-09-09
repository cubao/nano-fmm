#pragma once

#include <unordered_map>
#include <string>
#include <iostream>

namespace nano_fmm {
    struct Indexer {
        bool index(const std::string &str_id, int64_t int_id) {
            if (str2int_.count(str_id) || int2str_.count(int_id)) {
                return false;
            }
            str2int_.emplace(str_id, int_id);
            int2str_.emplace(int_id, str_id);
            return true;
        }
        std::string id(int64_t id) {
            auto itr = int2str_.find(id);
            if (itr != int2str_.end()) {
                return itr->second;
            }
            int round = 0;
            auto id_str = std::to_string(id);
            auto str_id = id_str;
            while (str2int_.count(str_id)) {
                ++round;
                str_id = id_str + "/" + std::to_string(round);
            }
            index(str_id, id);
            return str_id;
        }
        int64_t id(const std::string &id) {
            auto itr = str2int_.find(id);
            if (itr != str2int_.end()) {
                return itr->second;
            }
            try {
                int64_t value = std::stoll(id);
            } catch (const std::invalid_argument& ia) {
            } catch (const std::out_of_range& oor) {
            }
            return 0;
        }

        private:
        std::unordered_map<std::string, int64_t> str2int_;
        std::unordered_map<int64_t, std::string> int2str_;
        int64_t id_cursor_{1000000};
    };
}