#ifndef MAP_H
#define MAP_H

#include <Arduino.h>

/* Handle association */
template<typename hash, typename map>
class HashType {
public:
  HashType() {
    reset();
  }
  HashType(hash code, map value)
    : hashCode(code), mappedValue(value) {}

  // Reset hash and value
  void reset() {
    hashCode = hash();  // Verwende Standardkonstruktor
    mappedValue = map();
  }

  // Getter and setter for hash
  hash getHash() const {
    return hashCode;
  }
  void setHash(hash code) {
    hashCode = code;
  }

  // Getter and setter for value
  map getValue() const {
    return mappedValue;
  }
  void setValue(map value) {
    mappedValue = value;
  }

  // Operator for setting hash and value
  HashType& operator()(hash code, map value) {
    setHash(code);
    setValue(value);
    return *this;
  }

private:
  hash hashCode;
  map mappedValue;
};

/* Handle indexing and searches */
template<typename hash, typename map>
class Map {
public:
  // Constructor to initialize map with a given size
  Map(uint8_t newSize) {
    hashMap = new HashType<hash, map>[newSize];
    mapSize = newSize;
    for (uint8_t i = 0; i < mapSize; i++) {
      hashMap[i].reset();
    }
  }

  // Destructor to free memory
  ~Map() {
    delete[] hashMap;
    hashMap = nullptr;  // Avoid dangling pointers
  }

  // Access operator for indexing
  HashType<hash, map>& operator[](uint8_t x) {
    return hashMap[x % mapSize];
  }

  // Find the index of a given key, or return mapSize as an error
  uint8_t indexOf(hash key) const {
    for (uint8_t i = 0; i < mapSize; i++) {
      if (hashMap[i].getHash() == key) {
        return i;
      }
    }
    return mapSize;  // Error value: key not found
  }

  // Get the value associated with a given key
  map valueOf(hash key, map defaultValue = map()) const {
    for (uint8_t i = 0; i < mapSize; i++) {
      if (hashMap[i].getHash() == key) {
        return hashMap[i].getValue();
      }
    }
    return defaultValue;  // Return user-defined or default value
  }

  // Get the size of the map
  uint8_t getSize() const {
    return mapSize;
  }

private:
  HashType<hash, map>* hashMap;  // Pointer to the array of HashType
  uint8_t mapSize;               // Size of the map
};

#endif
