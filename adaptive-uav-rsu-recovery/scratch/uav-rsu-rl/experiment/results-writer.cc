#include "results-writer.h"

#include <cmath>
#include <sys/stat.h>

namespace uavrsu
{

CsvRow&
CsvRow::operator<<(double value)
{
    if (!m_first)
    {
        m_os << ",";
    }
    m_first = false;
    if (std::isfinite(value))
    {
        m_os << std::setprecision(6) << value;
    }
    else
    {
        m_os << "nan";
    }
    return *this;
}

CsvRow&
CsvRow::operator<<(bool value)
{
    return *this << (value ? 1 : 0);
}

std::string
CsvRow::Str() const
{
    return m_os.str();
}

bool
CsvWriter::Open(const std::string& path, const std::vector<std::string>& header)
{
    bool needHeader = true;
    {
        std::ifstream probe(path, std::ios::ate);
        needHeader = !probe || probe.tellg() == 0;
    }
    m_out.open(path, std::ios::app);
    if (!m_out)
    {
        return false;
    }
    if (needHeader)
    {
        for (std::size_t i = 0; i < header.size(); ++i)
        {
            m_out << (i ? "," : "") << header[i];
        }
        m_out << "\n";
    }
    return true;
}

void
CsvWriter::Write(const CsvRow& row)
{
    if (m_out)
    {
        m_out << row.Str() << "\n";
    }
}

void
CsvWriter::Close()
{
    if (m_out.is_open())
    {
        m_out.close();
    }
}

bool
CsvWriter::IsOpen() const
{
    return m_out.is_open();
}

std::vector<std::string>
RunContext::Header()
{
    return {"run_id", "label", "mode", "strategy", "algorithm", "scenario", "trace_variant",
            "num_vehicles", "seed", "run", "failed_rsu", "episode"};
}

void
RunContext::AppendTo(CsvRow& row) const
{
    row << runId << label << mode << strategy << algorithm << scenario << traceVariant << numVehicles
        << seed << run << failedRsu << episode;
}

std::vector<std::string>
Concat(std::vector<std::string> a, const std::vector<std::string>& b)
{
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

bool
EnsureDirectory(const std::string& path)
{
    std::string partial;
    for (std::size_t i = 0; i <= path.size(); ++i)
    {
        if (i == path.size() || path[i] == '/')
        {
            if (!partial.empty())
            {
                mkdir(partial.c_str(), 0755);
            }
        }
        if (i < path.size())
        {
            partial += path[i];
        }
    }
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

} // namespace uavrsu
