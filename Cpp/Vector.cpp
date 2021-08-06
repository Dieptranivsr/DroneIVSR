// Example program
#include <iostream>
#include <string>
#include <vector>

int main()
{
  std::vector<double> proc_ranges = { 1, 5, 6, 0, 63, 0};
  std::vector<double> zero_mat;
	for (unsigned int i = 0; i < proc_ranges.size(); i++)
		if (proc_ranges.at(i) == 0)
			zero_mat.push_back(i);
	for (auto i : zero_mat)
	    std::cout << i << std::endl;

    std::vector<double> row1;
    row1.push_back(1.0); row1.push_back(2.0); row1.push_back(3.0);

    std::vector<double> row2;
    row2.push_back(4.0); row2.push_back(5.0); row2.push_back(6.0); row2.push_back(7.0);

    std::vector<std::vector<double> > vector_of_rows;
    vector_of_rows.push_back(row1);
    vector_of_rows.push_back(row2);

    for( auto i : vector_of_rows){
        for( auto j : i)
            std::cout << j ;
        std::cout << std::endl;
    }
}
