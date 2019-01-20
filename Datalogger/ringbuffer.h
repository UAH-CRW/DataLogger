#ifndef RINGBUFFER_H_INCLUDED
#define RINGBUFFER_H_INCLUDED

template <class T>
class RingBuffer
{
  private:
    uint16_t array_length;
    uint16_t head;
    uint16_t tail;
    T* buffer;
  public:
    RingBuffer(T* backing_array, uint16_t backing_array_length);
    void write(T* data, uint16_t length);
    T* read(T* dest, uint16_t length);
    void delete_oldest(uint16_t length);
    T get_nth(uint16_t index);
    uint16_t length();
}

RingBuffer::RingBuffer(T* backing_array, uint16_t backing_array_length)
{
  buffer = backing_array;
  array_length = backing_array_length;
  head = 0;
  tail = 0;
}

RingBuffer::length()
{
  /*  Returns the number of bytes of data currently stored in the RingBuffer pointed to by its argument
  The return value will always be greater than or equal to 0
  And less than or equal to buffer->array_length
*/
{
  if (head >= tail)
  //We are not currently wrapped around the end of the buffer, so we are using the space between the head and tail
  {
    return head - tail;
  }
  else
  //We have wrapped around the end of the buffer, so subtract the unused space between the head and tail
  {
    return array_length - (tail - head);
  }
}

RingBuffer::write(T* data, uint16_t length)
/* Adds length bytes, taken from the data argument, to the end of buffer */
// Author: William Hankins
{
  for (uint16_t i = 0; i < length; i++)
  {
    buffer[head] = data[i]; // data can be not be access greater length-1
    head++;
    
    if (head == array_length) //Next value to write to is beyond the end of the array
    {
      head = 0;
    } // end if
    
    if (head == tail)
    {
      tail++;
      if (tail == array_length){
        tail = 0;
      }// end if
    } // end if
  } // end for
}

RingBuffer::read((T* dest, uint16_t length))
/*  Reads length bytes of data from the bottom of buffer.
  dest - resulting data will be stored there. Must be at least length bytes long
  length - number of bytes to be read
  Return values
  * 0 - success
  * 1 - the buffer doesn't have length bytes of data in it, but dest now has everything that was in there, null-terminated
*/
{
  uint16_t index = tail;
  uint16_t i = 0;
  for (i = 0; i < min(this->length(), length); i++)
  {
    dest[i] = buffer[index];
    if (index == array_length - 1) //Reset if we've hit the end
    {
      index = 0;
    }
    else
    {
      index++;
    }
  }
  if (length > this->length())
    return 1;
  else
    return 0;
}

void RingBuffer::delete_oldest(uint16_t length)
/*  Deletes data from the ring buffer
  All it really has to do is move buffer->tail up length bytes or until one byte below buffer->head, whichever is lower
*/
{
  uint16_t move_distance = min(length, this->length());
  if (move_distance >= this->length())
  {
    tail = (tail + move_distance) % array_length; //Modulus is so that we don't point to above the buffer's location
    head = tail;
  }
  else
  {
    tail = (tail + move_distance) % array_length; //Modulus is so that we don't point to above the buffer's location
  }
}

T RingBuffer::get_nth(uint16_t index)
//Returns the nth newest value in the RingBuffer
{
  //index = 0: gets the newest value
  //index = 1: gets the 2nd-newest value
  //etc
  //Does not do bounds checking, so be careful in usage
  //  Verify index < rbu8_length to guard against requesting a value beyond what the buffer currently stores
  //  Verify index < buffer->array_length to guard against requesting a value from some random memory location
  
  //Most recent value is in head - 1
  if (index + 1 <= head) //Value is between zero and buffer->head
    return buffer[head - index - 1];
  else
    return buffer[array_length - 1 - index + head];
}

#endif RINGBUFFER_H_INCLUDED
