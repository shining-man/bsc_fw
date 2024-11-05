// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTPERIPHERAL_H
#define EXTPERIPHERAL_H

class ExtPeripheral {
  protected:
      bool enabled = false;

      virtual void initialize() = 0;  // Initialisiert das Gerät

      void setEnabled(bool enable) { enabled = enable; }

  public:
      virtual ~ExtPeripheral() = default;  // Virtueller Destruktor für saubere Speicherfreigabe

      bool isEnabled() const { return enabled; }
};

#endif