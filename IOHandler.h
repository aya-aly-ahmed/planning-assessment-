/*
This file contains all the functions that are responsible for reading/writing data to the data files.
*/

/*
    Adjust this code to skip the comment lines and take a BONUS!
    OR leave it as is and delete the comment lines in the CSV file yourself (no BONUS!)
  */
vector<vector<float>> readCSV(const string& filename) {
    vector<vector<float>> data;
    ifstream file(filename);
    string line;

    while (getline(file, line)) {
        // Skip comment lines and empty lines
        string trimmed = line;
        trimmed.erase(0, trimmed.find_first_not_of(" \t\n\r"));
        if (trimmed.empty() || trimmed[0] == '#') continue;

        stringstream lineStream(line);
        string cell;
        vector<float> row;
        while (getline(lineStream, cell, ',')) {
            // Remove whitespace
            size_t start = cell.find_first_not_of(" \t\n\r");
            size_t end = cell.find_last_not_of(" \t\n\r");
            if (start == string::npos || end == string::npos) continue;
            string clean = cell.substr(start, end - start + 1);
            try {
                row.push_back(stof(clean));
            } catch (...) {
                // Ignore non-numeric cells
            }
        }
        if (!row.empty())
            data.push_back(row);
    }
    return data;
}


/*This function is responsible for writing the final path, which is an vector/ordered list of IDs.*/
void writePath(const vector<int>& data, const string& filename) {
    ofstream file(filename);

    for (const auto& cell : data) {
            file << cell;
            file << ',';
    }
    file << '\n';
}

/*This function is responsible for writing the edges between your nodes, which are ordered pairs of IDs.*/
void writeEdges(const std::string& filename, const std::vector<vector<int>>& edges) {
    std::ofstream output_file(filename);

    if (output_file.is_open()) {
        for (const vector<int>& edge : edges) {
            std::stringstream ss;
            ss << edge[0] << "," << edge[1] <<"\n";
            output_file << ss.str();
        }
        output_file.close();
    } else {
        std::cerr << "Error opening file: " << filename << std::endl;
    }
}

/*This function is responsible for appending the newly added nodes, which are in the form of ID,x,y.*/
void appendNodes(const std::string& filename, std::vector<std::vector<float>>& data) {
    std::ofstream output_file(filename, std::ios::app);

    if (output_file.is_open()) {
        output_file << "\n";
        for (vector<float>& row : data) {
            std::stringstream ss;
            
            
            for (size_t i = 0; i < row.size(); ++i) {
                if (i == 0)
                {
                    ss << to_string((int)row[i]) << ",";
                    continue;
                }
                
                ss << to_string(row[i]) << ",";
            }
            ss.seekp(-1, std::ios::cur); // Remove trailing comma
            ss << "\n";
            output_file << ss.str();
        }
        output_file.close();
    } else {
        std::cerr << "Error opening file: " << filename << std::endl;
    }
}
