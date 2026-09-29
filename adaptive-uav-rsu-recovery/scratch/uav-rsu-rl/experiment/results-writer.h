#ifndef UAVRSU_RESULTS_WRITER_H
#define UAVRSU_RESULTS_WRITER_H

// EXPERIMENT LAYER - CSV output helpers.
// Files are opened in append mode and the header is written only when the
// file is new, so repeated runs/episodes (training loops, parameter sweeps)
// accumulate into the same research CSVs. Every row starts with the same
// run-context columns so results from different strategies, densities and
// seeds can be grouped directly in pandas/matplotlib.

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace uavrsu
{

class CsvRow
{
  public:
    template <typename T>
    CsvRow& operator<<(const T& value)
    {
        if (!m_first)
        {
            m_os << ",";
        }
        m_first = false;
        m_os << value;
        return *this;
    }

    CsvRow& operator<<(double value);
    CsvRow& operator<<(bool value);
    std::string Str() const;

  private:
    std::ostringstream m_os;
    bool m_first = true;
};

class CsvWriter
{
  public:
    bool Open(const std::string& path, const std::vector<std::string>& header);
    void Write(const CsvRow& row);
    void Close();
    bool IsOpen() const;

  private:
    std::ofstream m_out;
};

/** Columns shared by every CSV file (identify the run a row belongs to). */
struct RunContext
{
    std::string runId;
    std::string label;
    std::string mode;
    std::string strategy;
    std::string algorithm;
    std::string scenario;
    std::string traceVariant;
    uint32_t numVehicles = 0;
    uint32_t seed = 1;
    uint32_t run = 1;
    int32_t failedRsu = -1;
    uint32_t episode = 0; // global episode index of this simulation run

    static std::vector<std::string> Header();
    void AppendTo(CsvRow& row) const;
};

std::vector<std::string> Concat(std::vector<std::string> a, const std::vector<std::string>& b);

bool EnsureDirectory(const std::string& path);

} // namespace uavrsu

#endif
