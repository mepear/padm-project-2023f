; This is a comment line
(define (domain kitchen)
  (:requirements :strips)
  (:predicates
    (close_to_desk)
    (sugar_on_stove)
    (sugar_in_hand)
    (spam_on_counter)
    (spam_in_hand)
    (sugar_on_counter)
    (spam_in_drawer)
    (drawer_open)
  )
  (:action move
    :precondition (not (close_to_desk))
    :effect (close_to_desk)
  )
  (:action open_drawer
    :precondition (and
      (not (sugar_in_hand))
      (not (spam_in_hand)) 
      (not (drawer_open))
      (close_to_desk)
    )
    :effect (drawer_open)
  )
  (:action close_drawer
    :precondition (and
      (not (sugar_in_hand))
      (not (spam_in_hand)) 
      (drawer_open)
      (close_to_desk)
    )
    :effect (not (drawer_open))
  )
  (:action pick_sugar
    :precondition (and
      (sugar_on_stove)
      (close_to_desk)
      (not (spam_in_hand))
      (not (sugar_in_hand))      
    )
    :effect (and
      (not (sugar_on_stove))
      (sugar_in_hand)
    )
  )  
  (:action place_sugar
    :precondition (and
      (sugar_in_hand)
      (close_to_desk)
    )
    :effect (and
      (not (sugar_in_hand))
      (sugar_on_counter)
    )
  )
  (:action pick_spam
    :precondition (and
      (spam_on_counter)
      (close_to_desk)
      (not (spam_in_hand))
      (not (sugar_in_hand))      
    )
    :effect (and
      (not (spam_on_counter))
      (spam_in_hand)
    )
  )  
  (:action place_spam
    :precondition (and
      (spam_in_hand)
      (drawer_open)
      (close_to_desk)
    )
    :effect (and
      (not (spam_in_hand))
      (spam_in_drawer)
    )
  )
)