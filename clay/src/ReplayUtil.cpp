#include "StdAfx.h"
#include "ReplayUtil.h"

#include <iostream>
#include <fstream>

//#include <boost/filesystem.hpp>

const char* ReplayUtil::configFilename = "ReplayUtil.cfg";
const char* ReplayUtil::historyFilename = "ReplayHistory.dat";

ReplayUtil::ReplayUtil()
{
  initMode();
}

ReplayUtil::~ReplayUtil()
{

}

void ReplayUtil::initMode()
{
  std::ifstream file(configFilename);

  // Does config file exist
  if (file.good()) {
    // Determine mode
    char buffer[1024];
    file >> buffer;
    if (0 == std::strcmp(buffer, "RECORD")) {
      setMode(MODE_RECORD);
    } else if (0 ==std::strcmp(buffer, "REPLAY")) {
      setMode(MODE_REPLAY);
    } else {
      setMode(MODE_INACTIVE);
    }
    file.close();
  } else {
    // Create the file
    std::ofstream file(configFilename);
    file << "RECORD" << std::endl;
    file.close();
    setMode(MODE_RECORD);
  }
}

void ReplayUtil::setMode(Mode mode) {
  m_mode = mode;
  m_history.clear();
  m_historyIdx = 0;
  m_historyStream.clear();
  m_historyStream.reserve(1024*10);
  m_historyStreamIdx = 0;

  //delete m_historyIStream; 
  //delete m_historyOStream;
  //m_historyIStream = NULL;
  //m_historyOStream = NULL;
  //m_historyStreamBuf.

  switch(m_mode) {
  case MODE_RECORD:
    clearHistoryFile();
    break;
  case MODE_REPLAY:
    loadHistory();
    break;
  case MODE_INACTIVE:
    // off
    break;
  }
}

void ReplayUtil::loadHistory()
{
  std::ifstream file(historyFilename, std::ios::binary);
#if LM_USE_ITEMS
  Item item;
  file >> item;
  while (file.good()) {
    // Create item & push
    m_history.push_back(item);
    file >> item;
  }
#else
  // Use stream
  file.seekg(0, std::ios::end);
  int size = (int)file.tellg();
  m_historyStream.resize(size);
  file.seekg(0, std::ios::beg);
  if (m_historyStream.size())
  {
    file.read(&m_historyStream[0], size);
  }
  file.close();
#endif
}

void ReplayUtil::clearHistoryFile() {
  std::ofstream file(historyFilename, std::ios::binary | std::ios::trunc);
  if (!file) {
    std::cerr << "could not truncate" << std::endl;
  }
  file.flush();
  file.close();
}

void ReplayUtil::saveHistory()
{
  std::ofstream file(historyFilename, std::ios::binary | std::ios::app);
#if LM_USE_ITEMS
  for (unsigned i = 0; i < m_history.size(); i++)
  {
    file << m_history[i];
  }
  m_history.clear();
#else
  if (m_historyStream.size())
  {
    file.write(&m_historyStream[0], m_historyStream.size());
  }
  m_historyStream.clear();
  m_historyStream.reserve(1024*10);
#endif

}

void ReplayUtil::addItem( const Item& item )
{
  m_history.push_back(item);
}

const ReplayUtil::Item& ReplayUtil::getItem()
{
  // assert in bounds
  return m_history[m_historyIdx++];
}

void ReplayUtil::trackItem(Item& item) {
  if (m_mode == MODE_RECORD) {
    addItem(item);
    saveHistory();
  } else if (m_mode == MODE_REPLAY) {
    item = getItem();
  }
}


//template <typename T>
//void ReplayUtil::trackValue(T& value) {
//  Item container;
//  *reinterpret_cast<T*>(&container) = value;
//  trackItem(container);
//  value = *reinterpret_cast<T*>(&container);
//}
//
//template <typename T>
//void ReplayUtil::trackConstValue(const T& value) {
//  Item container;
//  *reinterpret_cast<T*>(&container) = value;
//  trackItem(container);
//  const_cast<T&>(value) = *reinterpret_cast<T*>(&container);
//}

//template void ReplayUtil::trackValue(float& item);
//template void ReplayUtil::trackValue(int& item);
//
//template void ReplayUtil::trackConstValue(const float& item);
//template void ReplayUtil::trackConstValue(const int& item);


std::ostream& operator << (std::ostream& os, const ReplayUtil::Item& i) {
  os.write(reinterpret_cast<const char*>(&i), sizeof(i));
  return os;
}

std::istream& operator >> (std::istream& is, ReplayUtil::Item& i) {
  is.read(reinterpret_cast<char*>(&i), sizeof(i));
  return is;
}
