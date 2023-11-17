; This is a comment line
(define (domain kitchen)
  (:requirements :strips)
  (:predicates
    (close_to_desk)
    (sugar_on_stove)
    (spam_on_counter)
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
      (not (drawer_open))
      (close_to_desk)
    )
    :effect (drawer_open)
  )
  (:action close_drawer
    :precondition (and
      (drawer_open)
      (close_to_desk)
    )
    :effect (not (drawer_open))
  )
  (:action carry_sugar_to_counter
    :precondition (and
      (sugar_on_stove)
      (close_to_desk)
    )
    :effect (and
      (not (sugar_on_stove))
      (sugar_on_counter)
    )
  )
  (:action carry_spam_to_drawer
    :precondition (and 
      (drawer_open)
      (spam_on_counter)
      (close_to_desk)
    )
    :effect (and
      (not (spam_on_counter))
      (spam_in_drawer)
    )
  )
)