/**
 * Adds a new message to the activeMessages array.
 * 
 * @param option The index of the status message to be added.
 */
void addMessage(int option)
{
  int newData[4];

  // Copy the status message to newData
  for (int i = 0; i < 4; i++)
  {
    newData[i] = statusMessages[option][i];
  }

  // Check if there is space in the activeMessages array
  if (messageQty < maxMsg)
  {
    // Add the new message to activeMessages
    for (int i = 0; i < 4; i++)
    {
      activeMessages[messageQty][i] = newData[i];
    }
    Serial.print("Adding new message: ");
    Serial.print(option);
    Serial.print(",");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(newData[i]);
    }
    Serial.println();
    messageQty++;
  }
  else
  {
    // Serial.println("Message queue is full");
  }
}

/**
 * @brief Deletes a message from the statusMessages array based on the given option.
 * 
 * This function removes a message from the statusMessages array by comparing it with the targetData.
 * If a matching message is found, it is removed from the activeMessages array.
 * 
 * @param option The index of the message to be removed.
 */
void delMessage(int option)
{
  // Get the message to be removed
  int targetData[4];
  for (int i = 0; i < 4; i++)
  {
    targetData[i] = statusMessages[option][i];
  }
  Serial.print("Deleting message: ");
  Serial.print(option);
  Serial.print(",");
  for (int i = 0; i < 4; i++)
  {
    Serial.print(targetData[i]);
  }
  Serial.println();

  for (int i = 0; i < messageQty; i++)
  {
    Serial.println(compareArrays(activeMessages[i], targetData));
    if (compareArrays(activeMessages[i], targetData))
    {
      // If the current message matches the targetData, remove it
      for (int j = i; j < messageQty - 1; j++)
      {
        for (int k = 0; k < 4; k++)
        {
          activeMessages[j][k] = activeMessages[j + 1][k];
        }
      }
      Serial.println("Compare found an equal");
      messageQty--;
      break; // Break out of the loop after removing the first matching message
    }
    else
    {
      Serial.println("Compare found no equal");
    }
  }
}

/**
 * Compares two arrays of integers.
 * 
 * @param array1 The first array to compare.
 * @param array2 The second array to compare.
 * @return True if the arrays are identical, false otherwise.
 */
bool compareArrays(const int array1[4], const int array2[4])
{
  for (int i = 0; i < 4; i++)
  {
    if (array1[i] != array2[i])
    {
      return false; // Arrays are different
    }
  }
  return true; // Arrays are identical
}

void rotateMessages()
{
  unsigned long currentTime = millis();

  // Check if the display change interval has passed
  if (currentTime - lastDisplayChangeTime >= displayChangeInterval)
  {
    // Increment the error message index
    currentMessageIndex = (currentMessageIndex + 1) % messageQty;

    // Set the display with the new error message
    for (int i = 0; i < 4; i++)
    {
      // Serial.println(activeMessages[currentMessageIndex][i]);
    }
    setDisplayBits(activeMessages[currentMessageIndex]);

    // Update the last display change time
    lastDisplayChangeTime = currentTime;
  }
}