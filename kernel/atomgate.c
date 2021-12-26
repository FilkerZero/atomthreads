/*
 * Copyright (c) 2017, Daniel Glasser. All rights reserved.
 * Portions Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file atomgate.c
 * Atomthreads Gate library.
 *
 * This module implements a lightweight gate synchronization object to the
 * atomthreads Run Time Executive. A "gate" in this context is a binary
 * execution control mechanism that is similar to a transistor or relay
 * in electronics.  A gate can be in one of two states based on the value
 * of a control level (analogous to the control input to the transistor).
 * When the level is high, some calling threads can continue, others
 * will be blocked. Same is true when the level is low. Each time the
 * level value that controls the state of the gate object changes, all
 * threads previously blocked on it are allowed to proceed. At any one
 * time, all threads blocked on a gate object are waiting for the state
 * opposite its current state.
 *
 * The control value of the gate is restricted to two levels, low and
 * high.  Threads that call the interface to wait on a gate specify
 * a desired control level and a timeout.  If the desired control level
 * matches that of the gate object, the thread continues without blocking,
 * otherwise it is suspended and inserted into the blocking queue for
 * the gate.  When the state changes, when the "release" interface is
 * called, or when the gate object is deleted, all threads currently
 * waiting on the gate object are unblocked.  If a blocked threads
 * timeout expires, that thread is unblocked individually without
 * affecting any remaining threads blocked on the gate object.
 *
 * The gate model differs from the semaphore and mutex in several key ways.
 * Both the mutex and semaphore provide a serial control model based on
 * limiting the number of threads that may access some resource (code
 * execution, data structures, IO devices, etc.).  The gate, on the other
 * hand, is a parallel control model.  When the state of the gate changes,
 * all threads blocked on that gate are unblocked and eligible for
 * scheduling.  Typically, a gate is tied to an event (software or hardware)
 * that multiple threads are waiting for; when the event occurs, all of those
 * threads get notified at the same time.
 *
 * In its simplest form, the gate can be used as an on-off switch, where
 * all threads that pass through it desire the same control state.  Allowing
 * threads to block on either the high or low state is a small extension,
 * adding only the choice of which state to wait for and which state to
 * release.  Waiting for the high state is like a PNP transistor, electrons
 * flow when a positive (high) signal is applied to the control input.
 * Similarly, waiting for the low state is like an NPN transistor, allowing
 * flow only when the control input is low.  Using both control states at
 * once is a set of PNP and NPN transistors with the control input tied to
 * the same signal. This is also the model of a mechanical relay or switch
 * with two circuits where only one is open and one closed for each state.
 *
 * Gates do not have problems with priority inversion, since no thread may
 * own a gate, the a change affects all tasks blocked on the gate.  Since the
 * gate only permits two control states, all threads blocked are waiting for
 * whatever state the gate is not currently in.
 *
 * The following public interfaces are defined by the gate library:
 *
 *    uint8_t atomGateCreate(ATOM_GATE* gate, uint8_t level);
 *    uint8_t atomGateDelete(ATOM_GATE* gate);
 *    uint8_t atomGateWaitFor(ATOM_GATE*, uint8_t waitFor, int32_t timeout);
 *    uint8_t atomGateSetControl(ATOM_GATE* gate, uint8_t level);
 *    uint8_t atomGateRelease(ATOM_GATE* gate, uint8_t waitingFor);
 *    uint8_t atomGateGetControl(ATOM_GATE* gate, uint8_t* levelPtr);
 *
 * atomGateCreate()
 *    Initialize n gate object with a specified control level.
 *    Allocation is done by the caller.
 *
 * atomGateDelete()
 *    Unblocks all threads blocked on the gate and resets the gate control
 *    state to low.  Doesn't actually delete the gate object, as that
 *    is managed by the client software.  It's a bad practice to use
 *    a deleted gate, however doing so is the same as using a newly
 *    created gate.
 *
 * atomGateWaitFor()
 *    Conditionally suspends the calling thread on the gate when the
 *    gate is not in the desired state and the specified timeout.
 *    Timeout is supported to resume execution if the state does not
 *    change within a specified period.
 *
 * atomGateSetControl()
 *    Sets the control state of a gate.  If the state is changed, all threads
 *    blocked on the gate are unblocked.  If the state of the gate object is
 *    not changed, no threads are unblocked.
 *
 * atomGateRelease()
 *    Unblocks all threads that are waiting on the gate object if the current
 *    state does not match the requested released state without changing the
 *    state of the gate object. Can be used to release all blocked threads
 *    without regard to what state they're blocked on.  This prevents a race
 *    condition that can otherwise occur between control state changes.
 *
 * atomGateGetControl()
 *    Retrieves the control state level property of the gate object without
 *    affecting anything else.  Note that the value retrieved is only a sample
 *    and if an interrupt or scheduling preemption occurs between when the
 *    sample is taken and when that sample is evaluated by the application
 *    code, the actual state of the gate object may have changed.
 *
 * All of the public functions exposed by this library module return
 * atomthreads status values (see individual function documentation for
 * specific details), with ATOM_OK meaning success.
 *
 * Examples:
 * 1) Push button activation.
 *    An application has actions that are performed when the user presses
 *    and releases a pushbutton.  The pushbutton is connected to an input
 *    that generates an interrupt when the button is released.  The
 *    application also has functions that are performed continuously
 *    whether the button is pressed or not.
 *
 *    With the gate, one or more threads that implement the push-button
 *    action wait for a high control level on the gate. The threads that
 *    implement the background (continuous) functions do not wait on the
 *    gate.  When the pushbutton is pressed, the interrupt handler sets the
 *    control state of the gate to high, unblocking the button action threads.
 *    On return from the interrupt, the RTE scheduler runs the higher priority
 *    button action threads (or uses "round robin" scheduling if at the same
 *    priority level); when the button action is completed, the action threads
 *    set the gate control level to low and then wait on the gate.  If the
 *    action thread(s) set the gate control state back to low immediately on
 *    release from the gate, a second push of the button before the action is
 *    completed will cause the action to repeat immediately, though pressing
 *    the button 3 times while the first time through the action is running
 *    will not cause the action to repeat that many times.  Setting the state
 *    of the gate at the end helps debounce the input.
 *
 *    Alternatively, the button interrupt can release the threads waiting on
 *    the gate without changing the state, relieving the action threads of the
 *    need to set the control level to low before waiting on the gate again.
 *    Additional button interrupts before the action threads are blocked again
 *    not affect the control state of the gate, so the action will not repeat
 *    until the user presses the button after the action has completed.
 *    Even in this scenario, if there are multiple foreground threads that
 *    wait on the same gate, they will not all complete and block again at the
 *    same time, and a button press between the first and last foreground
 *    threads blocking would cause a premature unblocking of a subset of the
 *    foreground action threads.  It is up to the software designer to solve
 *    this.
 *
 * 2) Service Completion
 *    An asynchronous operation is performed by a service thread that only
 *    runs while performing the operation.  A single gate can be used to
 *    start the operation and signal completion.  The requesting thread sets
 *    up the parameters for the operation, sets the control state of the gate,
 *    goes on to do other things, then waits, as before, until the state
 *    changes.  The same gate object is used by both the requesting and
 *    service threads, each blocking on the opposite control state.  This
 *    interaction between the requesting and service threads is generally
 *    wrapped up in an API layer that hides the details from the requesting
 *    thread implementation.
 *
 * 3) DMA or Interrupt driven serial IO
 *    Platform provided device drivers often provide an asynchronous transmit
 *    API that takes a buffer pointer and a byte count as arguments, sets up
 *    the operation, and returns before the IO has completed.  The driver
 *    performs the output using DMA or interrupts working out of the buffer
 *    provided by the caller.  This allows very efficient use of the compute
 *    resources of the system, but can create obscure problems when the
 *    support libraries do not use alternate buffers.  The POSIX conforming
 *    libraries provided by most vendors use a single per-thread, per-stream
 *    buffer for "*printf()" operations, and start at the beginning of the
 *    buffer after each call to the underlying "write()" or "write_r"
 *    function.  If that underlying function returns before the output has
 *    completed, the library will overwrite part of the buffer possibly
 *    before the bytes from that part of the buffer have been transferred
 *    to the output device, causing the data seen on the serial line to be
 *    corrupted.  Using the driver APIs that do not return until the
 *    transmit is complete generally will consume processor resources polling
 *    the output device, thus are not efficent in a multi-threaded system.
 *
 *    The original implementation of the gate for atomthreads was created to
 *    solve this problem.  The function that makes the call to the device
 *    driver sets the state of a gate, calls the driver, then waits for the
 *    state of the gate to change back before returning to the caller
 *    (in this case, the "printf()" primitive functions in "newlib").
 *    A transmit complete interrupt callback function changes the state of
 *    the gate object, thus releasing the thread.
 *
 *    The thread cannot overlap execution with the output, but the thread
 *    is blocking on the gate and consuming no CPU cycles until the
 *    transmit-done callback is invoked from the driver's interrupt handler.
 *
 *    That original version of the gate was called "event", and was later
 *    generalized into the "gate" construct implementation.
 *
 * Acknowledgments:
 *  * The source for this module was started using a copy of the code in
 *    the file file "atommutex.c", written as part of the atomthreads
 *    distribution by Kelvin Lawson.  Though much of the code is substantially
 *    different, some of the code is still just a minor modification of the
 *    original.
 */
#include "atom.h"
#include "atomtimer.h"
#include "atomgate.h"

typedef struct gate_timer
{
   ATOM_TCB*  tcb_ptr;  /* A thread waiting for an gate */
   ATOM_GATE* gate_ptr; /* The gate the thread is waiting on */
} GATE_TIMER;

static void atomGateTimerCallback(POINTER cb_data);

/**
 * \b atomGateCreate
 *
 * Initializes an gate object.
 *
 * Must be called before calling any other gate library routines on an
 * gate. Gate objects can be "deleted" later using atomGateDelete().
 *
 * Does not allocate storage, the caller provides the gate object.
 *
 * This function can be called from interrupt context (not sure why
 * that would be done).
 *
 * @param[in] gate Pointer to the gate object to be initialized
 * @param[in] level Initial control level of the gate, one of: \n
 *             atomGate_Low  (0): Gate control is low
 *             atomGate_High (1): Gate control is high
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 */
uint8_t atomGateCreate(ATOM_GATE* gate, uint8_t level)
{
   uint8_t status;

   /* Parameter check */
   if ((gate == NULL) || (level >= atomGate_NoChange))
   {
      /* Bad gate pointer */
      status = ATOM_ERR_PARAM;
   }
   else
   {
      /* Initialize the suspended threads queue */
      gate->suspQ = NULL;
      gate->level = level;

      /* Successful */
      status = ATOM_OK;
   }

   return status;
} /* end of atomGateCreate() */

/**
 * \b atomGateReleaseInternal
 *
 * Internal function to release all threads pending for a control state on a
 * gate object, optionally changing the gate control level.  This function may
 * not be called from outside this source module; it is called by the public
 * interfaces "atomGateDelete()", "atomGateSetControl()", and
 * "atomGateRelease()".
 *
 * At any given time, all threads suspended on a gate are waiting for the same
 * control state.  The caller specifies which control state to release, and if
 * the gate level property is not in that state, all threads on the suspend
 * queue for the gate are moved to the run queue and the scheduler is called.
 * The caller may request that all threads waiting on the gate be unblocked
 * regardless of the control state.
 *
 * The caller also selects how the state of the gates level property is
 * affected.  It may be left unchanged, toggled, or be set to a specific
 * high or low state.
 *
 * If any threads are unblocked, this function calls the scheduler, which may
 * result in the calling thread being preempted before this function returns
 * when called from thread context.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the gate, so the potential execution
 * cycles cannot be determined in advance.
 *
 * @param[in] gate Pointer to gate object
 * @param[in] threadStatus Status to set thread return values to
 * @param[in] waitingOn  The controls state to release threads blocked on,
 *            one of: \n
 *               atomGate_Low    (0): Waiting on control level low \n
 *               atomGate_High   (1): Waiting on control level high \n
 *               atomGate_All    (2): Unconditional
 * @param[in] newLevel Selects how to affect the the control state of the
 *            gate, one of: \n
 *               atomGate_Low      (0): Set control level low \n
 *               atomGate_High     (1): Set control level high \n
 *               atomGate_NoChange (2): Do not modify control level \n
 *               atomGate_Toggle   (3): Toggle the control level
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem canceling a timeout on a woken thread
 * @retval ATOM_ERR_PARAM Problem with a parameter
 */
static uint8_t atomGateReleaseInternal(ATOM_GATE* gate,
                                       uint8_t threadStatus,
                                       uint8_t waitingOn,
                                       uint8_t newLevel)
{
   uint8_t status;
   CRITICAL_STORE;
   ATOM_TCB *tcb_ptr;
   uint8_t woken_threads = FALSE;

   /* Parameter check */
   if ((gate == NULL)               ||
       (waitingOn > atomGate_All)   ||
       (newLevel > atomGate_Toggle))
   { /* a parameter failed validation */
      status = ATOM_ERR_PARAM;
   }
   else
   { /* valid parameters */
      /* Default to success status unless errors occur during wakeup */
      status = ATOM_OK;

      /* Enter critical region */
      CRITICAL_START();

      /* Conditionally update the control state level for the gate
       */
      if (newLevel < atomGate_NoChange)
      {
         gate->level = newLevel;
      }
      else if (newLevel == atomGate_Toggle)
      {
         gate->level = !gate->level;
      }

      if (waitingOn == atomGate_All)
      {
         waitingOn = gate->level;
      }


      /* Wake up all suspended tasks */
      while (waitingOn == gate->level)
      {
         /* Check if any threads are suspended on this gate */
         tcb_ptr = tcbDequeueHead(&gate->suspQ);

         /* At least one thread is suspended on the gate */
         if (NULL != tcb_ptr)
         {
            /* Return 'threadStatus' status to the waiting thread */
            tcb_ptr->suspend_wake_status = threadStatus;

            /* Put the thread on the ready queue */
            if (tcbEnqueuePriority(&tcbReadyQ, tcb_ptr) != ATOM_OK)
            {
               /* Exit critical region */
               CRITICAL_END();

               /* Quit the loop, returning error */
               status = ATOM_ERR_QUEUE;
               break;
            }

            /* If there's a timeout on this suspension, cancel it */
            if (tcb_ptr->suspend_timo_cb)
            {
               /* Cancel the callback */
               if (atomTimerCancel(tcb_ptr->suspend_timo_cb) != ATOM_OK)
               {
                  /* Exit critical region */
                  CRITICAL_END();

                  /* Quit the loop, returning error */
                  status = ATOM_ERR_TIMER;
                  break;
               }

               /* Flag as no timeout registered */
               tcb_ptr->suspend_timo_cb = NULL;
            }

            /* Exit critical region */
            CRITICAL_END();

            /* Request a reschedule */
            woken_threads = TRUE;
         } /* end of if (tcb_ptr...) */
         else
         { /* No more suspended threads */
            /* Exit critical region and exit the loop */
            CRITICAL_END();
            break;
         }
      }

      /* Call scheduler if any threads were woken up */
      if (woken_threads == TRUE)
      {
         /**
          * Only call the scheduler if we are in thread context, otherwise
          * it will be called on exiting the ISR by atomIntExit().
          */
         if (atomCurrentContext())
         {
            atomSched(FALSE);
         }
      }
   }

   return status;
} /* end of atomGateReleaseInternal() */

/**
 * \b atomGateDelete
 *
 * Deletes an gate object (actually, simply releases).
 *
 * Any threads currently suspended on the gate will be woken up with
 * return status ATOM_ERR_DELETED. If called at thread context then the
 * scheduler will be called during this function which may schedule in one
 * of the woken threads depending on relative priorities.  The control state
 * of the gate is set to low.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the gate, so the potential execution
 * cycles cannot be determined in advance.
 *
 * @param[in] gate Pointer to gate object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem canceling a timeout on a woken thread
 * @retval ATOM_ERR_PARAM Problem with the parameter (NULL)
 */
uint8_t atomGateDelete(ATOM_GATE* gate)
{
   uint8_t status;

   /* Unblock all threads waiting on this gate with a return status of
    * ATOM_ERR_DELETED.  The control state of the gate is set to low,
    * which is arbitrary but consistent.
    */
   status = atomGateReleaseInternal(gate, ATOM_ERR_DELETED,
                                    atomGate_All, atomGate_Low);

   return status;
} /* end of atomGateDelete() */


/**
 * \b atomGateSetControl
 *
 * Sets the control state of the gate
 *
 * If the control state changes, any threads suspended on the gate will be
 * unblocked up with return status ATOM_OK.  When called from thread context
 * and one or more threads are unblocked, the scheduler will be called, which
 * may result in the calling thread being preempted before the function
 * returns.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the gate, so the potential execution
 * cycles cannot be determined in advance.
 *
 * @param[in] gate Pointer to gate object
 * @param[in] newLevel Selects the desired control state for the gate object,
 *            one of: \n
 *              atomGate_Low    (0)
 *              atomGate_High   (1)
 *              atomGate_Toggle (3)
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem canceling a timeout on a woken thread
 * @retval ATOM_ERR_PARAM Problem with the parameter (NULL)
 */
uint8_t atomGateSetControl(ATOM_GATE* gate, uint8_t newLevel)
{
   uint8_t status;

   if (newLevel != atomGate_NoChange)
   { /* valid 'newLevel' */
      uint8_t release;
      if (newLevel == atomGate_Toggle)
      {
         release = atomGate_All;
      }
      else
      {
         release = newLevel;
      }
      /* Release all threads pending on this gate and clear the gate flag */
      status = atomGateReleaseInternal(gate, ATOM_OK, release, newLevel);
   }
   else
   {
      status = ATOM_ERR_PARAM;
   }
   return status;
} /* end of atomGateSetControl() */


/**
 * \b atomGateRelease
 *
 * Releases all threads pending for a control state on a gate object without
 * changing the control state for the gate.
 *
 * Any threads unblocked from the suspend queue of the gate object will have
 * the return status ATOM_OK.  When called from thread context and one or
 * more threads are unblocked, the scheduler will be called, which may
 * result in the calling thread being preempted before the function returns.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the gate, so the potential execution
 * cycles cannot be determined in advance.
 *
 * @param[in] gate Pointer to gate object
 * @param[in] waitingOn  The controls state to release threads blocked on,
 *            one of: \n
 *               atomGate_Low    (0): Waiting on control level low \n
 *               atomGate_High   (1): Waiting on control level high \n
 *               atomGate_All    (2): Unconditional
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem canceling a timeout on a woken thread
 * @retval ATOM_ERR_PARAM Problem with the parameter (NULL)
 */
uint8_t atomGateRelease(ATOM_GATE* gate, uint8_t waitingOn)
{
   uint8_t status;

   if (waitingOn <= atomGate_All)
   {
      status = atomGateReleaseInternal(gate, ATOM_OK,
                                       waitingOn, atomGate_NoChange);
   }
   else
   {
      status = ATOM_ERR_PARAM;
   }
   return status;
} /* end of atomGateRelease() */

/**
 * \b atomGateGetControl
 *
 * Query the current control state of a gate object
 *
 * This function can be called from interrupt context.  It has no scheduling
 * consequence.
 *
 * @param[in] gate Pointer to gate object
 * @param[out] levelPtr Pointer to a location to store the control level
 *             value for the gate object
 *
 * @retval ATOM_OK Success, *levelPtr will be updated
 * @retval ATOM_ERR_PARAM Bad pointer, *levelPtr is not accessed
 */
uint8_t atomGateGetControl(ATOM_GATE* gate, uint8_t* levelPtr)
{
   uint8_t status;

   /* Parameter check */
   if ((gate == NULL) || (levelPtr == NULL))
   { /* parameter error */
      status = ATOM_ERR_PARAM;
   }
   else
   { /* neither parameter is NULL */
      status = ATOM_OK;
      *levelPtr = gate->level;
   }

   return status;
} /* end of atomGateGetControl() */

/**
 * \b atomGateWaitFor
 *
 * Conditionally suspend the calling thread based on the current level of
 * a gate objects control value.
 *
 * If the desired control state does not match the level property of the
 * gate object, the calling thread is suspended pending a change in the
 * control level for the gate.  If the gate is already in the desired
 * control state, the function returns immediately.
 *
 * Depending on the \c timeout value specified the call will do one of
 * the following:
 *
 * \c timeout == 0 : Call will block until the gate is signalled. \n
 * \c timeout > 0 : Call will block until the gate is signalled, or the
 *                  specified timeout has elapsed \n
 * \c timeout == -1 : Return immediately if gate has been signalled \n
 *
 * If the call needs to block and \c timeout is non-zero, the call will only
 * block for the specified number of system ticks after which time, if the
 * thread was not already woken, the call will return with \c ATOM_TIMEOUT.
 *
 * If \c timeout is -1 and the desired state does not match the current
 * control state of the gate object, the call will return immediately
 * with status \c ATOM_WOULDBLOCK.
 *
 * This function may only be called from thread context.
 *
 * @param[in] gate Pointer to an gate object
 * @param[in] waitFor  The control state desired for the thread to proceed;
 *            one of: \n
 *               atomGate_Low    (0): Wait for control level low \n
 *               atomGate_High   (1): Wait for control level high \n
 *               atomGate_All    (2): Wait for control state change
 * @param[in] timeout Max system ticks to block (0 = forever)
 *
 * @retval ATOM_OK Success
 * @retval ATOM_TIMEOUT Gate timed out before being signalled
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but count is zero
 * @retval ATOM_ERR_DELETED Gate was deleted while thread was suspended
 * @retval ATOM_ERR_CONTEXT Not called in thread context
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 */
uint8_t atomGateWaitFor(ATOM_GATE *gate,
                        uint8_t waitFor,
                        int32_t timeout)
{
   CRITICAL_STORE;
   uint8_t status = ATOM_OK;
   GATE_TIMER timer_data;
   ATOM_TIMER timer_cb;
   ATOM_TCB *curr_tcb_ptr;

   /* Check parameters */
   if ((gate == NULL) || (waitFor > atomGate_All))
   { /* Bad parameter value */
      status = ATOM_ERR_PARAM;
   }
   else
   { /* valid looking parameters */
      /* Get the current TCB */
      curr_tcb_ptr = atomCurrentContext();

      /* If the desired control state (in 'waitFor') is 'atomGate_All',
       * make it the inverse of the current control state level
       */
      if (atomGate_All == waitFor)
      { /* wait for state change, don't care which way */
         waitFor = !gate->level;
      }

      /* Protect access to the gate object and OS queues */
      CRITICAL_START();

      /**
       * Check we are at thread context.  It is never valid to call this
       * function from an interrupt / exception handler regardless of
       * whether it will block.
       */
      if (curr_tcb_ptr == NULL)
      {
         /* Exit critical region */
         CRITICAL_END();

         /* Not currently in thread context, can't suspend */
         status = ATOM_ERR_CONTEXT;
      }
      else if ((timeout == -1) && (gate->level != waitFor))
      {
         CRITICAL_END();
         status = ATOM_WOULDBLOCK;
      }
      else if (gate->level == waitFor)
      {
         CRITICAL_END();
         status = ATOM_OK;
      }
      else if (timeout >= 0)
      { /* enqueue */
         /* If called with timeout >= 0, we should block */
         /* Add current thread to the suspend list on this gate */
         if (tcbEnqueuePriority(&gate->suspQ, curr_tcb_ptr) != ATOM_OK)
         {
            /* Exit critical region */
            CRITICAL_END ();

            /* There was an error putting this thread on the suspend list */
            status = ATOM_ERR_QUEUE;
         }
         else
         { /* successfully enqueued */
            /* Set suspended status for the current thread */
            curr_tcb_ptr->suspended = TRUE;

            /* Track errors */
            status = ATOM_OK;

            /* Register a timer callback if requested */
            if (timeout > 0)
            {
               /* Fill out the data needed by the callback to wake us up */
               timer_data.tcb_ptr = curr_tcb_ptr;
               timer_data.gate_ptr = gate;

               /* Fill out the timer callback request structure */
               timer_cb.cb_func = atomGateTimerCallback;
               timer_cb.cb_data = (POINTER)&timer_data;
               timer_cb.cb_ticks = timeout;

               /**
                * Store the timer details in the TCB so that we can
                * cancel the timer callback if the gate is put
                * before the timeout occurs.
                */
               curr_tcb_ptr->suspend_timo_cb = &timer_cb;

               /* Register a callback on timeout */
               if (atomTimerRegister(&timer_cb) != ATOM_OK)
               {
                  /* Timer registration failed */
                  status = ATOM_ERR_TIMER;

                  /* Clean up and return to the caller */
                  (void)tcbDequeueEntry(&gate->suspQ, curr_tcb_ptr);
                  curr_tcb_ptr->suspended = FALSE;
                  curr_tcb_ptr->suspend_timo_cb = NULL;
               }
            }
            else
            { /* Set no timeout requested */
               /* No need to cancel timeouts on this one */
               curr_tcb_ptr->suspend_timo_cb = NULL;
               status = ATOM_OK;
            }

            /* Exit critical region */
            CRITICAL_END();

            /* Check no errors have occurred */
            if (status == ATOM_OK)
            { /* ready to suspend this thread */
               /**
                * Current thread now blocking, schedule in a new
                * one. We already know we are in thread context
                * so can call the scheduler from here.
                */
               atomSched(FALSE);

               /**
                * Normal atomGateRaise() wakeups will set ATOM_OK status,
                * while timeouts will set ATOM_TIMEOUT and gate deletions
                * will set ATOM_ERR_DELETED. 
                */
               status = curr_tcb_ptr->suspend_wake_status;
            } /* thread was blocked */
         } /* successfully enqueued */
      } /* enqueue */
   } /* Non-NULL gate pointer */

   return status;
} /* end of atomGateWaitFor() */

/**
 * \b atomGateTimerCallback
 *
 * This is an internal function not for use by application code.
 *
 * Timeouts on suspended threads are notified by the timer system through
 * this generic callback. The timer system calls us back with a pointer to
 * the relevant \c GATE_TIMER object which is used to retrieve the
 * gate details.
 *
 * @param[in] cb_data Pointer to a GATE_TIMER object
 */
static void atomGateTimerCallback (POINTER cb_data)
{
   GATE_TIMER *timer_data_ptr;
   CRITICAL_STORE;

   /* Get the GATE_TIMER structure pointer */
   timer_data_ptr = (GATE_TIMER *)cb_data;

   /* Check parameter is valid */
   if (timer_data_ptr)
   {
      /* Enter critical region */
      CRITICAL_START();

      /* Set status to indicate to the waiting thread that it timed out */
      timer_data_ptr->tcb_ptr->suspend_wake_status = ATOM_TIMEOUT;

      /* Flag as no timeout registered */
      timer_data_ptr->tcb_ptr->suspend_timo_cb = NULL;

      /* Remove this thread from the gate's suspend list */
      (void)tcbDequeueEntry(&timer_data_ptr->gate_ptr->suspQ,
                            timer_data_ptr->tcb_ptr);

      /* Put the thread on the ready queue */
      (void)tcbEnqueuePriority(&tcbReadyQ, timer_data_ptr->tcb_ptr);

      /* Exit critical region */
      CRITICAL_END();

      /**
       * Note that we don't call the scheduler now as it will be called
       * when we exit the ISR by atomIntExit().
       */
   }
} /* end of atomGateTimerCallback() */
