# enum
uint8 STATUS_NORMAL=1  # Successfull data acquisition
uint8 STATUS_ERROR=2   # Failed to acquire data
uint8 SEAWATER=1  # Successfull data acquisition
uint8 FRESHWATERFRESHWATER=2   # Failed to acquire data

#msg
float32 depth
float32 temperature
uint8 status
uint8 watertype